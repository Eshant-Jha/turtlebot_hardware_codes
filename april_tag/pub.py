#!/usr/bin/env python
import rospy
import math
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

if __name__ == '__main__':
    rospy.init_node('tag_publisher')

    tfBuffer0 = tf2_ros.Buffer()    
    listener0 = tf2_ros.TransformListener(tfBuffer0)

    tfBuffer1 = tf2_ros.Buffer()    
    listener1 = tf2_ros.TransformListener(tfBuffer1)

    tfBuffer2 = tf2_ros.Buffer()    
    listener2 = tf2_ros.TransformListener(tfBuffer2)

    # Publisher for the apriltag_seven topic
    pub0 = rospy.Publisher('/apriltag_zero', Odometry, queue_size=2)
    pub1 = rospy.Publisher('/apriltag_one', Odometry, queue_size=2)
    pub2 = rospy.Publisher('/apriltag_two', Odometry, queue_size=2)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            current_time = rospy.Time.now()

            try:
                transformation0 = tfBuffer0.lookup_transform('origin', 'zero', rospy.Time())
            # Create an Odometry message for tag 0
                odometry_msg_0 = Odometry()
                odometry_msg_0.header.stamp = rospy.Time.now()
                odometry_msg_0.header.frame_id = 'origin'
                odometry_msg_0.child_frame_id = 'zero'
                odometry_msg_0.pose.pose.position = Point(
                    x=transformation0.transform.translation.x,
                    y=transformation0.transform.translation.y,
                    z=transformation0.transform.translation.z
                )
                odometry_msg_0.pose.pose.orientation = transformation0.transform.rotation
                odometry_msg_0.twist.twist = Twist()

                pub0.publish(odometry_msg_0)
                print(odometry_msg_0)

            except:
                print("no tag zero")
            
            try:
                transformation1 = tfBuffer1.lookup_transform('origin', 'one', rospy.Time())

                odometry_msg_1 = Odometry()
                odometry_msg_1.header.stamp = rospy.Time.now()
                odometry_msg_1.header.frame_id = 'origin'
                odometry_msg_1.child_frame_id = 'one'
                odometry_msg_1.pose.pose.position = Point(
                    x=transformation1.transform.translation.x,
                    y=transformation1.transform.translation.y,
                    z=transformation1.transform.translation.z
                )
                odometry_msg_1.pose.pose.orientation = transformation1.transform.rotation
                odometry_msg_1.twist.twist = Twist()

                pub1.publish(odometry_msg_1)
                print(odometry_msg_1)
            except:
                print("no tag one")

            try:

                transformation2 = tfBuffer2.lookup_transform('origin', 'two', rospy.Time())
                odometry_msg_2 = Odometry()
                odometry_msg_2.header.stamp = rospy.Time.now()
                odometry_msg_2.header.frame_id = 'origin'
                odometry_msg_2.child_frame_id = 'two'
                odometry_msg_2.pose.pose.position = Point(
                    x=transformation2.transform.translation.x,
                    y=transformation2.transform.translation.y,
                    z=transformation2.transform.translation.z
                )
                odometry_msg_2.pose.pose.orientation = transformation2.transform.rotation
                odometry_msg_2.twist.twist = Twist()

                pub2.publish(odometry_msg_2)
                print(odometry_msg_2)

            except:
                print("no tag two")


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
            print("execpted")
            rate.sleep()
            continue

        rate.sleep()

    rospy.spin()  # Add rospy.spin() to keep the node running
