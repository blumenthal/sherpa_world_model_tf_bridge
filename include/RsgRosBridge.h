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

#ifndef RSGROSBRIDGE_H_
#define RSGROSBRIDGE_H_

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>

/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateSerializer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateDeserializer.h>

namespace brics_3d {
namespace rsg {

class RsgRosOutputBridge : public brics_3d::rsg::IOutputPort {
public:
	RsgRosOutputBridge(ros::NodeHandle node, std::string topicName);
	virtual ~RsgRosOutputBridge();

	void initialize();
	int write(const char* dataBuffer, int dataLength, int& transferredBytes);

private:

	/// The node handle used for the publisher
	ros::NodeHandle node;

	/// Name of the topic
	std::string topicName;

	/// Publisher used for the updates
	ros::Publisher hdf5StreamPublisher;

	/// Timer for benchmarking.
	Timer timer;
};

class RsgRosInputBridge {
public:
	RsgRosInputBridge(brics_3d::rsg::IInputPort* inputPort, ros::NodeHandle node, std::string topicName);
	virtual ~RsgRosInputBridge();

	void initialize();

private:

	void updateCallback(const std_msgs::ByteMultiArray::ConstPtr &msg);

	/// Interface to a port that handles the un-mashaling and update of the world model
	brics_3d::rsg::IInputPort* inputPort;

	/// The node handle used for the subsriber
	ros::NodeHandle node;

	/// Name of the topic
	std::string topicName;

	/// Subscriber used to recieve the updates
	ros::Subscriber hdf5StreamSubscriber;

	/// Timer for benchmarking.
	Timer timer;
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSGROSBRIDGE_H_ */

/* EOF */
