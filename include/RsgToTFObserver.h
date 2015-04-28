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

#ifndef RSG_RsgToTFObserver_H_
#define RSG_RsgToTFObserver_H_

/* RSG related headers */
#include <brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h>
#include <brics_3d/worldModel/sceneGraph/Attribute.h>
#include <brics_3d/util/Timer.h>

/* ROS TF related headers */
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "SceneGraphTransformNodes.h"
#include "SceneGraphTypeCasts.h"

namespace brics_3d {
namespace rsg {

class RsgToTFObserver : public ISceneGraphUpdateObserver {
public:
	RsgToTFObserver(WorldModel* wm);
	virtual ~RsgToTFObserver();

	/* implemetntations of observer interface */
	bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false);
    bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId = false);
	bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false);
	bool addRemoteRootNode(Id rootId, vector<Attribute> attributes);
	bool addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId = false);
	bool setNodeAttributes(Id id, vector<Attribute> newAttributes);
	bool setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
    bool setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp);
	bool deleteNode(Id id);
	bool addParent(Id id, Id parentId);
    bool removeParent(Id id, Id parentId);


private:

    void processTransformUpdate (std::string frameId);
    bool tfNodeExistsInWorldModel(std::string frameId);
    brics_3d::rsg::Id getTfNodeByFrameId(std::string frameId);
    brics_3d::rsg::Id getParentNodeByFrameId(std::string frameId);

    tf::TransformBroadcaster tfPublisher;

	std::map <std::string, SceneGraphTransformNodes> sceneGraphToTfMapping;

	/// Turn on/off to automatically create corresponding RSG nodes based on all so far recieved TF frames.
	bool enableFrameAutoDiscovery;

	WorldModel* wm;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_RsgToTFObserver_H_ */

/* EOF */
