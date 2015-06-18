/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2014, KU Leuven
*
* Author: Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/


#include <iostream>
#include <fstream>

/* ROS includes */
#include <ros/ros.h>
//#include <tf/tf.h>
//#include <tf/transform_listener.h> //@deprecated
//#include <tf/transform_broadcaster.h> //@deprecated
#include <tf2_bullet/tf2_bullet/tf2_bullet.h> // package.xml does not resolve this correctly
#include <LinearMath/btTransform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/ByteMultiArray.h> // update channel for world model

/* BRICS_3D includes */
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/SceneGraphFacade.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateSerializer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateDeserializer.h>
#include <brics_3d/worldModel/sceneGraph/OutdatedDataIdAwareDeleter.h>
#include <brics_3d/worldModel/sceneGraph/SceneGraphToUpdatesTraverser.h>
#include <brics_3d/worldModel/sceneGraph/FrequencyAwareUpdateFilter.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>

/* BRICS_3D <-> ROS bindings*/
#include "RsgRosBridge.h"
#include "RsgToTFObserver.h"
#include "SceneGraphTransformNodes.h"

/* Application specific includes */
#include <boost/algorithm/string.hpp>

/* Optional visialization via OSG. This has to be enabled via a CMake option */
#ifdef ENABLE_OSG
	#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#endif

using brics_3d::Logger;
using namespace brics_3d::rsg;

//inline static void convertTfTransformToHomogeniousMatrix (const tf::Transform& tfTransform, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix)
//{
//	double mv[12];
//	tfTransform.getBasis().getOpenGLSubMatrix(mv);
//	tf::Vector3 origin = tfTransform.getOrigin();
//	double* matrixPtr = transformMatrix->setRawData();
//
//	/* matrices are column-major */
//	matrixPtr[0] = mv[0]; matrixPtr[4] = mv[4]; matrixPtr[8] = mv[8]; matrixPtr[12] = origin.x();
//	matrixPtr[1] = mv[1]; matrixPtr[5] = mv[5]; matrixPtr[9] = mv[9]; matrixPtr[13] = origin.y();
//	matrixPtr[2] = mv[2]; matrixPtr[6] = mv[6]; matrixPtr[10] = mv[10]; matrixPtr[14] = origin.z();
//	matrixPtr[3] = 0; matrixPtr[7] = 0; matrixPtr[11] = 0; matrixPtr[15] = 1;
//}

inline static void convertTfTransformToHomogeniousMatrix (const tf2::Stamped<btTransform>& tfTransform, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix)
{
	btScalar mv[12];
	tfTransform.getBasis().getOpenGLSubMatrix(mv);
	btVector3 origin = tfTransform.getOrigin();
	double* matrixPtr = transformMatrix->setRawData();

	/* matrices are column-major */
	matrixPtr[0] = mv[0]; matrixPtr[4] = mv[4]; matrixPtr[8] = mv[8]; matrixPtr[12] = origin.x();
	matrixPtr[1] = mv[1]; matrixPtr[5] = mv[5]; matrixPtr[9] = mv[9]; matrixPtr[13] = origin.y();
	matrixPtr[2] = mv[2]; matrixPtr[6] = mv[6]; matrixPtr[10] = mv[10]; matrixPtr[14] = origin.z();
	matrixPtr[3] = 0; matrixPtr[7] = 0; matrixPtr[11] = 0; matrixPtr[15] = 1;
}


/**
 * @brief Class that encapsualtes the application.
 */
class WorldModelNode {
public:

	WorldModelNode(brics_3d::WorldModel* wm) : wm(wm) {

		graphResender = 0;

		/* Set up the deletion policy when to clean up parts of the world model. */
		garbageCollector = new OutdatedDataIdAwareDeleter(&wm->scene);
		garbageCollector->setEnableTransformDeletions(false);
		garbageCollector->setEnableGeometricNodeDeletions(true);

		/* TF specific init */
		maxTFCacheDuration = ros::Duration(1.0); //[s]

		/*
		 * Example for a manual setup.
		 */
//		/openni_depth_optical_frame
//	        frame_id: /base_footprint
//	        child_frame_id: /openni_camera
//		SceneGraphTransformNodes tmpSceneGraphTransformSpec;
//		tmpSceneGraphTransformSpec.id = INVALID_ID; // We will have to find out later.
//		tmpSceneGraphTransformSpec.name = "robot_to_sensor_tf";
//		tmpSceneGraphTransformSpec.tfParent = "/base_footprint";
//		std::string tmpFrameId = "/openni_depth_optical_frame";
//		tfToSceneGraphMapping.insert(std::make_pair(tmpFrameId, tmpSceneGraphTransformSpec));

		tfRootNode = "base_link";
		enableFrameAutoDiscovery = true;

		tfUpdateListener = new tf2_ros::TransformListener(tfListener);
		tfListener._addTransformsChangedListener(boost::bind(&WorldModelNode::processTfTopic, this));
	}

	virtual ~WorldModelNode() {
		delete garbageCollector;
		if (graphResender) {
			delete graphResender;
		}
		if (tfUpdateListener) {
			delete tfUpdateListener;
		}
	}

	void sceneSetup() {
		vector<Attribute> attributes;

		/*
		 * We initialize the world model with any frame_id as defined by tfRootNode
		 * e.g. "base_link" - which is commonly seen in TF trees.
		 * It serves as root for the world model as well.
		 */
		brics_3d::rsg::Id tfRootNodeId = 0;
		attributes.clear();
		attributes.push_back(Attribute("ros_tf:frame_id", tfRootNode));
		brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr initialBaseLinkTransform(new brics_3d::HomogeneousMatrix44());
		wm->scene.addTransformNode(wm->scene.getRootId(), tfRootNodeId, attributes, initialBaseLinkTransform, wm->now());

		SceneGraphTransformNodes tmpSceneGraphTransformSpec;
		tmpSceneGraphTransformSpec.id = tfRootNodeId;
		tmpSceneGraphTransformSpec.name = tfRootNode;
		tmpSceneGraphTransformSpec.tfParent = "NO_PARENT";
		std::string tmpFrameId = tfRootNode;
		tfToSceneGraphMapping.insert(std::make_pair(tmpFrameId, tmpSceneGraphTransformSpec));

//		/* Here comes a basic robot skeleton in case of a manual setup */
//		brics_3d::rsg::Id tfWorldToRobotId = 0;
//		attributes.clear();
//		attributes.push_back(Attribute("name","world_to_robot_tf"));
//		brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr initialWorldToRobotTransform(new brics_3d::HomogeneousMatrix44());
//		wm->scene.addTransformNode(wm->scene.getRootId(), tfWorldToRobotId, attributes, initialWorldToRobotTransform, wm->now());
//
//		brics_3d::rsg::Id tfRobotToSensorId = 0;
//		attributes.clear();
//		attributes.push_back(Attribute("name","robot_to_sensor_tf"));
//		brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr initialRobotToSensorTransform(new brics_3d::HomogeneousMatrix44());
//		wm->scene.addTransformNode(tfWorldToRobotId, tfRobotToSensorId, attributes, initialRobotToSensorTransform, wm->now());
//
//
//
//		/* We will add a hook for the processing data */
//		brics_3d::rsg::Id sensorGroupId;
//		attributes.clear();
//		attributes.push_back(Attribute("name","sensor"));
//		wm->scene.addGroup(tfRobotToSensorId, sensorGroupId, attributes);
//		LOG(DEBUG) << "Sensor group added with ID " << sensorGroupId;

	}

	void updateTfTopology() {
		/* Workaround based on string parsing */
		//		Frame robot_1 exists with parent wgs84.
		//		Frame robot_1_base_link exists with parent robot_1.
		//		Frame robot_2 exists with parent wgs84.
		std::stringstream  tfTreeAsString(tfListener.allFramesAsString());
		std::string  line;
		while(std::getline(tfTreeAsString, line)) {
//			LOG(DEBUG) << "tfTreeAsString " << line;
			std::vector<string> segments;
			boost::split(segments, line, boost::is_any_of(" "));
			string child  = segments[1];
			string parent = segments[5];
			parent.erase( parent.end()-1 ); //remove the the "." e.g. robot_1. to robot_1
			LOG(DEBUG) << "tfTreeAsStringSegments " << child << " has parent " << parent;
			if(tfChildToParentMap.find(child) == tfChildToParentMap.end()) {
				tfChildToParentMap.insert(std::make_pair(child, parent));
				LOG(DEBUG) << "Adding to tfChildToParentMap" << child << " has parent " << parent;
			}
		}
	}

	bool getTFParent(const std::string& frameId, ros::Time time, std::string& parent) {
		if(tfChildToParentMap.find(frameId) != tfChildToParentMap.end()) { // iff found, then it has a parent
			std::map <std::string, std::string>::iterator parentIter = tfChildToParentMap.find(frameId);
			parent = parentIter->second;
			return true;
		} else {
			return false;
		}
	}

	void addAllRecievedTFFramesToWorldModel() {
		std::vector<std::string> frameIds;
		tfListener._getFrameStrings(frameIds);
		updateTfTopology();

		/*
		 * Treverse in topological order:
		 * 1.) find root - the one without
		 */
		for (std::vector<std::string>::iterator iter = frameIds.begin(); iter != frameIds.end(); iter++) {
			LOG(INFO) << "FrameId: " << *iter;
			string parent = "unknown";

//			if(!tfListener._getParent(*iter, ros::Time::now(), parent)) { //this one does not work!
			if(!getTFParent(*iter, ros::Time::now(), parent)) { //this one does not work!
				LOG(INFO) << "\t is a root.";
			}
			LOG(DEBUG) << " has parent = " << parent;


			/* check if frame exists already */
			if (!tfNodeExistsInWorldModel(*iter)) {
				LOG(INFO) << "\t does not exist in RSG.";
				addNewTfToWorldModel(*iter);
			}
		}
		LOG(INFO) << tfListener.allFramesAsString();
		LOG(INFO) << tfListener._allFramesAsDot();

	}

	bool tfNodeExistsInWorldModel(std::string frameId) {
		return tfToSceneGraphMapping.find(frameId) != tfToSceneGraphMapping.end();
	}

	Id getTfNodeByFrameId(std::string frameId) {
		if (!tfNodeExistsInWorldModel(frameId)) {
			LOG(WARNING) << "getTfNodeByFrameId: a node with frameId " << frameId << " does not exist.";
			return Id(0); // Nil value
		}

		std::map <std::string, SceneGraphTransformNodes>::iterator iter = tfToSceneGraphMapping.find(frameId);
		return iter->second.id;

	}

	void addNewTfToWorldModel(std::string frameId) {
		LOG(DEBUG) << "addNewTfToWorldModel("<< frameId << ")";
		string parentId;

		/* Handle root node i.e. with "NO_PARENT" frame_id */
//		if(!tfListener._getParent(frameId, ros::Time::now(), parentId)) { // does not work
		if(!getTFParent(frameId, ros::Time::now(), parentId)) {
			LOG(INFO) << "addNewTfToWorldModel: " << frameId << " is a root node.";
			return;
		}

		/* check if parent exists already */
		if (tfNodeExistsInWorldModel(parentId)) {

			/* Just add it */
			brics_3d::rsg::Id wmFrameId = 0;
			vector<Attribute> attributes;
			attributes.push_back(Attribute("ros_tf:frame_id",frameId));
			brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr initialTransform(new brics_3d::HomogeneousMatrix44()); // gets anyways updated.
			if(wm->scene.addTransformNode(getTfNodeByFrameId(parentId), wmFrameId, attributes, initialTransform, wm->now())) {

				SceneGraphTransformNodes tmpSceneGraphTransformSpec;
				tmpSceneGraphTransformSpec.id = wmFrameId;
				tmpSceneGraphTransformSpec.name = frameId;
				tmpSceneGraphTransformSpec.tfParent = parentId;
				tfToSceneGraphMapping.insert(std::make_pair(frameId, tmpSceneGraphTransformSpec));

			}

		} else {

			/* Create parent as well */
			addNewTfToWorldModel(parentId);

		}
	}

	void processTfTopic () {
		if(enableFrameAutoDiscovery) {
			addAllRecievedTFFramesToWorldModel();
		}

		std::map <std::string, SceneGraphTransformNodes>::iterator iter = tfToSceneGraphMapping.begin();
		for (iter = tfToSceneGraphMapping.begin(); iter != tfToSceneGraphMapping.end(); iter++) {

			std::string tfFrameId = iter->first;
			std::string tfFrameReferenceId = iter->second.tfParent;
//			tf::StampedTransform transform;
//			tf2::Stamped<btTransform> transform;
			geometry_msgs::TransformStamped transform;
			try{
//				tfListener.lookupTransform(tfFrameReferenceId, tfFrameId, ros::Time(0), transform);
				transform = tfListener.lookupTransform(tfFrameReferenceId, tfFrameId, ros::Time(0));
			}
			catch (tf::TransformException ex){
				ROS_WARN("%s",ex.what());
				continue;
			}

			if ( (ros::Time::now() - transform.header.stamp) > maxTFCacheDuration ) { //simply ignore outdated TF frames
				ROS_WARN("TF found for %s. But it is outdated. Skipping it.", iter->first.c_str());
				continue;
			}
			ROS_INFO("TF found for %s.", iter->first.c_str());

			/*
			 * update scene graph
			 */

			/* resolve id if necessary */
			if (iter->second.id == INVALID_ID) {

				vector<Attribute> queryAttributes;
				queryAttributes.push_back(Attribute("name", iter->second.name));
				vector<Id> resultIds;

				if (!wm->scene.getNodes(queryAttributes, resultIds)) {
					ROS_ERROR("Failed to call service Get Node");
					continue;
				}

				ROS_INFO("Found %li IDs for node in the scene graph with name %s", resultIds.size(), iter->second.name.c_str());


				/* we attempt to take the first one (arbitrary choice) */
				if (resultIds.size() > 0) {
					iter->second.id = resultIds[0];
				} else {
					ROS_WARN("Cannot find a node in the scene graph with name %s", iter->second.name.c_str());
					continue;
				}
			}

			/* do the update */
			ROS_INFO("updating transform");
			brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transformUpdate(new brics_3d::HomogeneousMatrix44());
//			btTransform tmpBtTransform;
//			tf2::convert(transform, btTransform);
//			const geometry_msgs::TransformStamped constTransform = transform;
			btTransform tmpBtTransform = tf2::transformToBullet(transform);

			tf2::Stamped<btTransform> btStampedTransform;
			btStampedTransform.setData(tmpBtTransform);
			convertTfTransformToHomogeniousMatrix(btStampedTransform, transformUpdate);
			std::cout << *transformUpdate;

			if (!wm->scene.setTransform(iter->second.id, transformUpdate, wm->now())) {
				ROS_ERROR("Failed to call service Set Transform");
				continue;
			}


		}
	}

	void resendGraph() {

	}

	const std::string& getTfRootNode() const {
		return tfRootNode;
	}

	void setTfRootNode(const std::string& tfRootNode) {
		this->tfRootNode = tfRootNode;
	}

private:

	/// THE world model based on the Robot Scene Graph (RSG).
	brics_3d::WorldModel* wm;

	// Traversers
	OutdatedDataIdAwareDeleter* garbageCollector;

	/// Receives TF
//	tf::TransformListener tfListener;
	tf2_ros::Buffer tfListener;
	tf2_ros::TransformListener* tfUpdateListener;

	ros::Duration maxTFCacheDuration;

	/// Mapping
	std::map <std::string, SceneGraphTransformNodes> tfToSceneGraphMapping;

	/// Turn on/off to automatically create corresponding RSG nodes based on all so far recieved TF frames.
	bool enableFrameAutoDiscovery;

	/// Root not to be considered for automatic discovery.
	/// Typical values are "base_link", "map" or "odom"	.
	/// Deafault is "base_link".
	std::string tfRootNode;

	/// Store the topology of a td tree to. This ia a workaround to the tf API.
	std::map <std::string, std::string> tfChildToParentMap;

public:

	SceneGraphToUpdatesTraverser* graphResender;

};


int main(int argc, char **argv)
{

	/* Initialize ROS framework */
	ros::init(argc, argv, "world_model_tf_bridge");
	ros::NodeHandle node;


	/* Define logger level for world model */
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
	brics_3d::Logger::setLogfile("world_model_tf_bridge.log");

	/* Create an empty world model */
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();

	/* Attach additional debug output to the world model */
	brics_3d::rsg::DotVisualizer structureVisualizer(&wm->scene);
	wm->scene.attachUpdateObserver(&structureVisualizer);

	/* Connect input port to world model */
	brics_3d::rsg::HDF5UpdateDeserializer* inDeserializer = new brics_3d::rsg::HDF5UpdateDeserializer(wm);
	RsgRosInputBridge* inBridge = new RsgRosInputBridge(inDeserializer, node, "world_model/update_stream");

	/* Attach the outout filter + port */
	RsgRosOutputBridge* outBridge = new RsgRosOutputBridge(node, "world_model/update_stream_tf_bridge");
	HDF5UpdateSerializer* outSerializer = new HDF5UpdateSerializer(outBridge);
	outSerializer->setStoreMessageBackupsOnFileSystem(false);
	FrequencyAwareUpdateFilter* frequencyFilter = new FrequencyAwareUpdateFilter();
	frequencyFilter->setMaxTransformUpdateFrequency(1/*Hz*/);
	/* chaines up ubserver: wm -> frequencyFilter -> outSerializer */
	wm->scene.attachUpdateObserver(frequencyFilter);
	frequencyFilter->attachUpdateObserver(outSerializer);


	/* Initialize the application */
	WorldModelNode wmNode(wm);
	wmNode.graphResender = new SceneGraphToUpdatesTraverser(outSerializer);

#ifdef ENABLE_OSG
	//Visualization tool for world model
	brics_3d::rsg::OSGVisualizer* wmObserver = new brics_3d::rsg::OSGVisualizer(); // can segfault
	wm->scene.attachUpdateObserver(wmObserver); // enable visualization
	wm->scene.advertiseRootNode(); 				// required by visualizer
#endif

	wmNode.setTfRootNode("base_link"); // TODO param server
	wmNode.setTfRootNode("wgs84");
//	wmNode.setTfRootNode("world");
//	wmNode.setTfRootNode("robot_1");
	wmNode.sceneSetup();

	RsgToTFObserver rsgToTf(wm);
	//wm->scene.attachUpdateObserver(&rsgToTf);
	frequencyFilter->attachUpdateObserver(&rsgToTf);

	LOG(INFO) << "Ready.";

	/* Let the node loop and process messages. */
	ros::MultiThreadedSpinner spinner(2);
	wm->scene.advertiseRootNode();

	//spinner.spin();

	ros::Rate rate(0.2); // (in Hz)
	while (node.ok()){
		ros::spinOnce();
		wmNode.processTfTopic();
		LOG(INFO) << "Resending complete graph.";
		wmNode.graphResender->reset();
        wm->scene.executeGraphTraverser(wmNode.graphResender , wm->scene.getRootId());
		rate.sleep();
	}

	/* Clean up */
	delete wm;

	return 0;
}



/* EOF */
