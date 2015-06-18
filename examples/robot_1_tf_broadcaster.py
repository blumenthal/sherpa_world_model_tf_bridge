#!/usr/bin/env python  
# 
# Taken from tutorial http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
#
import rospy

# Because of transformations
import tf
import math

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('robot_1_tf')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() * math.pi / 10.0
	theta = -t * math.pi/5.0;
	q = tf.transformations.quaternion_from_euler(0, 0, theta)
        br.sendTransform((1.0 * math.sin(t), 2.0 * math.cos(t), (0.1 * math.sin(2*t))+ 0.2),
                         q,			
                         rospy.Time.now(),
                         "robot_1", # child
                         "wgs84")   # parent
	br.sendTransform((0.1, 0, 0),
                         (0, 0 ,0 ,1),			
                         rospy.Time.now(),
                         "robot_1_base_link", # child
                         "robot_1")   # parent
        rate.sleep()    	
