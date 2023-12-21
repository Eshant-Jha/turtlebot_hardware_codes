#!/usr/bin/env python
import rospy
import math
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

if __name__ == '__main__':
    rospy.init_node('tag_0_publisher')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Publisher for the apriltag_seven topic
    pub = rospy.Publisher('/apriltag_zero', Odometry, queue_size=10)

    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        try:
            transformation = tfBuffer.lookup_transform('origin', 'zero', rospy.Time())
            rospy.loginfo(transformation)  # Use loginfo instead of print for better ROS integration

            # Create an Odometry message
            odometry_msg = Odometry()
            odometry_msg.header.stamp = rospy.Time.now()
            odometry_msg.header.frame_id = 'origin'
            odometry_msg.child_frame_id = 'zero'
            odometry_msg.pose.pose.position = Point(
                x=transformation.transform.translation.x,
                y=transformation.transform.translation.y,
                z=transformation.transform.translation.z
            )
            odometry_msg.pose.pose.orientation = transformation.transform.rotation
            odometry_msg.twist.twist = Twist()

            # Publish the Odometry message
            pub.publish(odometry_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        rate.sleep()

    rospy.spin()  # Add rospy.spin() to keep the node running
