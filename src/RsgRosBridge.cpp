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

#include "RsgRosBridge.h"

namespace brics_3d {
namespace rsg {

RsgRosOutputBridge::RsgRosOutputBridge(ros::NodeHandle node, std::string topicName) :
	node(node), topicName(topicName) {

	initialize();
}

RsgRosOutputBridge::~RsgRosOutputBridge() {

}

void RsgRosOutputBridge::initialize() {
	hdf5StreamPublisher = node.advertise<std_msgs::ByteMultiArray>(topicName, 100);
}

int RsgRosOutputBridge::write(const char* dataBuffer, int dataLength,
		int& transferredBytes) {

	LOG(DEBUG) << "RsgRosOutputBridge: sendin new update message with " << dataLength << " bytes.";

	/* prepare ROS message */
	std_msgs::ByteMultiArray msg;
	msg.layout.data_offset = 0;
	msg.layout.dim.resize(1);
	msg.layout.dim[0].label = "raw byte data stream";
	msg.layout.dim[0].size = dataLength;
	msg.layout.dim[0].stride = msg.layout.dim[0].size; //?

	/* copy the byte data to ROS message*/
	msg.data.resize(msg.layout.dim[0].size);
	for (int i = 0; i < msg.layout.dim[0].size; ++i) {
		msg.data[i] = dataBuffer[i];
	}

	/* publish the ROS message*/
	timer.reset();
	hdf5StreamPublisher.publish(msg);
	TimeStamp duration(timer.getElapsedTime(), Units::MilliSecond);
	LOG(INFO) << "RsgRosOutputBridge: Publishing took " << std::setprecision(15) << duration.getSeconds() << " [s].";
}

} /* namespace rsg */
} /* namespace brics_3d */

brics_3d::rsg::RsgRosInputBridge::RsgRosInputBridge(
		brics_3d::rsg::IInputPort* inputPort, ros::NodeHandle node,
		std::string topicName) : inputPort(inputPort), node(node), topicName(topicName) {

	initialize();
}

brics_3d::rsg::RsgRosInputBridge::~RsgRosInputBridge() {

}

void brics_3d::rsg::RsgRosInputBridge::initialize() {
	int topicBufferSize = 100;
	hdf5StreamSubscriber = node.subscribe<std_msgs::ByteMultiArray>(topicName, topicBufferSize, &RsgRosInputBridge::updateCallback, this);
}

void brics_3d::rsg::RsgRosInputBridge::updateCallback(
		const std_msgs::ByteMultiArray::ConstPtr& msg) {

	int deseraializedBytes;
	int length = msg->layout.dim[0].size;
	char *buffer = new char [length];
	for (int idx = 0; idx < length; ++idx) {
		buffer[idx] = msg->data[idx];
	}

	timer.reset();
	inputPort->write(buffer, length, deseraializedBytes); // feed forward to the HDF5 deserialization
	TimeStamp duration(timer.getElapsedTime(), Units::MilliSecond);
	LOG(INFO) << "RsgRosInputBridge: Deserialization took " << std::setprecision(15) << duration.getSeconds() << " [s].";


}


/* EOF */
