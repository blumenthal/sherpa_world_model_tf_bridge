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

#include "RsgToTFObserver.h"
#include <brics_3d/core/Logger.h>


using brics_3d::Logger;

namespace brics_3d {
namespace rsg {

RsgToTFObserver::RsgToTFObserver() {

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

	return true;

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
		vector<Attribute> newAttributes) {

}

bool RsgToTFObserver::setTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp) {

	LOG(DEBUG) << "RsgToTFObserver::setTransform.";

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


} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
