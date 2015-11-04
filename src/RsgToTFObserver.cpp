/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2015, KU Leuven
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

#include "RsgToTFObserver.h"
#include <brics_3d/core/Logger.h>


using brics_3d::Logger;

namespace brics_3d {
namespace rsg {

RsgToTFObserver::RsgToTFObserver(WorldModel* wm) : wm(wm) {
	enableFrameAutoDiscovery = true;
}

RsgToTFObserver::~RsgToTFObserver() {

}

bool RsgToTFObserver::addNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {
	return true;
}

bool RsgToTFObserver::addGroup(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {

	return true;
}

bool RsgToTFObserver::addTransformNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp, bool forcedId) {

	LOG(DEBUG) << "RsgToTFObserver::addTransformNode.";

	string tag = "ros_tf:frame_id";
	vector<std::string> resultValues;
	if(getValuesFromAttributeList(attributes, tag, resultValues)) {
		string frameId = resultValues[0]; // >=1
		LOG(DEBUG) << "RsgToTFObserver: A new transform node has been added that belongs to the smantic conteixt of : " << tag;

		if(sceneGraphToTfMapping.find(frameId) == sceneGraphToTfMapping.end()) {

			LOG(DEBUG) << "RsgToTFObserver: Frame with fram_id  " << frameId << " not yet contained in mapping list.";

			/* get parent frame_id */
			vector<Attribute> parentAttributes;
			wm->scene.getNodeAttributes(parentId, parentAttributes);

			resultValues.clear();
			if(getValuesFromAttributeList(parentAttributes, tag, resultValues)) {
				string parentFrameId = resultValues[0]; // >=1

				LOG(DEBUG) << "RsgToTFObserver: parentFrameId is :" << parentFrameId;

				SceneGraphTransformNodes tmpSceneGraphTransformSpec;
				tmpSceneGraphTransformSpec.id = assignedId;
				tmpSceneGraphTransformSpec.name = frameId;
				tmpSceneGraphTransformSpec.tfParent = parentFrameId;
				sceneGraphToTfMapping.insert(std::make_pair(frameId, tmpSceneGraphTransformSpec));

				/* Immediatly push out this information */
				processTransformUpdate(frameId);

				return true;
			} else {
				LOG(WARNING) << "RsgToTFObserver: parentFrameId cannot be determined";
			}



		} else {
			LOG(DEBUG) << "RsgToTFObserver: Already conteined in mapping list.";
		}

	}

	return false;

}

bool RsgToTFObserver::addUncertainTransformNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp, bool forcedId) {

	return true;
}


bool RsgToTFObserver::addGeometricNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, Shape::ShapePtr shape,
		TimeStamp timeStamp, bool forcedId) {

	return true;
}

bool RsgToTFObserver::addRemoteRootNode(Id rootId, vector<Attribute> attributes) {

	return true;
}

bool RsgToTFObserver::addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId) {

	return true;
}

bool RsgToTFObserver::setNodeAttributes(Id id,
		vector<Attribute> newAttributes, TimeStamp timeStamp ) {

	return true;
}

bool RsgToTFObserver::setTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp) {

	LOG(DEBUG) << "RsgToTFObserver::setTransform.";


	string tag = "ros_tf:frame_id";
	vector<std::string> resultValues;
	vector<Attribute> attributes;

	wm->scene.getNodeAttributes(id, attributes);

	if(getValuesFromAttributeList(attributes, tag, resultValues)) {
		string frameId = resultValues[0]; // >=1
		processTransformUpdate(frameId);
		return true;
	}

	return false;
}

bool RsgToTFObserver::setUncertainTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp) {

	return true;
}

bool RsgToTFObserver::deleteNode(Id id) {

	return true;
}

bool RsgToTFObserver::addParent(Id id, Id parentId) {

	return true;
}

bool RsgToTFObserver::removeParent(Id id, Id parentId) {

	return true;
}

bool RsgToTFObserver::tfNodeExistsInWorldModel(std::string frameId) {
	return sceneGraphToTfMapping.find(frameId) != sceneGraphToTfMapping.end();
}

Id RsgToTFObserver::getTfNodeByFrameId(std::string frameId) {
	if (!tfNodeExistsInWorldModel(frameId)) {
		LOG(WARNING) << "getTfNodeByFrameId: a node with frameId " << frameId << " does not exist.";
		return Id(0); // Nil value
	}

	std::map <std::string, SceneGraphTransformNodes>::iterator iter = sceneGraphToTfMapping.find(frameId);
	return iter->second.id;
}

void RsgToTFObserver::processTransformUpdate (string frameId) {

	/* retrieve data from mapping */
	std::map <std::string, SceneGraphTransformNodes>::iterator it = sceneGraphToTfMapping.find(frameId);

	if(it == sceneGraphToTfMapping.end()) {
		LOG(ERROR) << "RsgToTFObserver::processTransformUpdate Frame with fram_id  " << frameId << " not contained in mapping list. Skipping it.";
	} else {

		geometry_msgs::TransformStamped tmpTransformMsg;
		brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr transformUpdate(new brics_3d::HomogeneousMatrix44());
		if(!wm->scene.getTransform(getTfNodeByFrameId(frameId), wm->now(), transformUpdate)) {
			LOG(WARNING) << "RsgToTFObserver::processTransformUpdate transform not found.";
			return;
		}
		SceneGraphTypeCasts::convertTransformToRosMsg(transformUpdate, tmpTransformMsg);

		/* add meta data */
		tmpTransformMsg.header.stamp = ros::Time::now();
		tmpTransformMsg.header.frame_id = it->second.tfParent;
		tmpTransformMsg.child_frame_id = it->second.name;

		/* push it out */
		LOG(ERROR) << "RsgToTFObserver::processTransformUpdate emitting  frame " << frameId  << std::endl << *transformUpdate;
		tfPublisher.sendTransform(tmpTransformMsg);
	}
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
