/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#ifndef SCENEGRAPHTYPECASTS_H_
#define SCENEGRAPHTYPECASTS_H_

/* BRICS_3D includes */
#include <brics_3d/worldModel/sceneGraph/SceneGraphFacade.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/Cylinder.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/Logger.h>

/* BRICS_3D <-> ROS types */

#include <tf/tf.h>

namespace brics_3d {

namespace rsg {

/**
 * @brief Helper class to cast between BRICS_3D and ROS mesage types
 */
class SceneGraphTypeCasts {
public:
	SceneGraphTypeCasts(){};
	virtual ~SceneGraphTypeCasts(){};

	inline static void convertTimeStampToRosMsg(brics_3d::rsg::TimeStamp& timeStamp, ros::Time& convertedTimeStamp) {

	}

	inline static void convertRosMsgToTimeStamp(const ros::Time& timeStamp, brics_3d::rsg::TimeStamp& convertedTimeStamp) { //here we _have_ to create a new timestamp
		brics_3d::rsg::TimeStamp tmpTimeStamp(timeStamp.toSec()*1000.0);
		convertedTimeStamp += tmpTimeStamp; //assumes default constructor;
	}

	inline static void convertTransformToRosMsg(brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform,  geometry_msgs::TransformStamped& convertedTransform){

		tf::Transform tmpTransform;
		convertHomogeniousMatrixToTfTransform(transform, tmpTransform);
		convertedTransform.transform.translation.x = tmpTransform.getOrigin().getX();
		convertedTransform.transform.translation.y = tmpTransform.getOrigin().getY();
		convertedTransform.transform.translation.z = tmpTransform.getOrigin().getZ();
		convertedTransform.transform.rotation.x = tmpTransform.getRotation().getX();
		convertedTransform.transform.rotation.y = tmpTransform.getRotation().getY();
		convertedTransform.transform.rotation.z = tmpTransform.getRotation().getZ();
		convertedTransform.transform.rotation.w = tmpTransform.getRotation().getW();
	}

	inline static void convertRosMsgToTransform(const geometry_msgs::TransformStamped transform, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& convertedTransform){
		tf::StampedTransform tmpTransform;
		tf::transformStampedMsgToTF(transform, tmpTransform);
		convertTfTransformToHomogeniousMatrix(tmpTransform, convertedTransform);
	}

	/* Some helper functions */
	inline static void convertTfTransformToHomogeniousMatrix (const tf::Transform& tfTransform, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix)
	{
		double mv[12];

		tfTransform.getBasis().getOpenGLSubMatrix(mv);
		tf::Vector3 origin = tfTransform.getOrigin();

		double* matrixPtr = transformMatrix->setRawData();

		/* matrices are column-major */
		matrixPtr[0] = mv[0]; matrixPtr[4] = mv[4]; matrixPtr[8] = mv[8];   matrixPtr[12] = origin.x();
		matrixPtr[1] = mv[1]; matrixPtr[5] = mv[5]; matrixPtr[9] = mv[9];   matrixPtr[13] = origin.y();
		matrixPtr[2] = mv[2]; matrixPtr[6] = mv[6]; matrixPtr[10] = mv[10]; matrixPtr[14] = origin.z();
		matrixPtr[3] = 0;     matrixPtr[7] = 0;     matrixPtr[11] = 0;      matrixPtr[15] = 1;

	}

	inline static void convertHomogeniousMatrixToTfTransform (const brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix, tf::Transform& tfTransform) {
		const double* matrixPtr = transformMatrix->getRawData();

		tf::Vector3 translation;
		tf::Matrix3x3 rotation;

		translation.setX(matrixPtr[12]);
		translation.setY(matrixPtr[13]);
		translation.setZ(matrixPtr[14]);

		rotation.setValue(
				matrixPtr[0], matrixPtr[4], matrixPtr[8],
				matrixPtr[1], matrixPtr[5], matrixPtr[9],
				matrixPtr[2], matrixPtr[6], matrixPtr[10]
		);

		tfTransform.setOrigin(translation);
		tfTransform.setBasis(rotation);
	}

	inline static bool convertPoseMsgToHomogeniousMatrix(geometry_msgs::PoseStamped pose, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transformMatrix) {
		tf::Transform tfTransform;
		tf::Vector3 translation;
		tf::Quaternion rotation;

		translation.setX(pose.pose.position.x);
		translation.setY(pose.pose.position.y);
		translation.setZ(pose.pose.position.z);
		rotation.setX(pose.pose.orientation.x);
		rotation.setY(pose.pose.orientation.y);
		rotation.setZ(pose.pose.orientation.z);
		rotation.setW(pose.pose.orientation.w);

		tfTransform.setOrigin(translation);
		tfTransform.setRotation(rotation);
		convertTfTransformToHomogeniousMatrix(tfTransform, transformMatrix);

		return true;
	}
};

}

}

#endif /* SCENEGRAPHTYPECASTS_H_ */

/* EOF */
