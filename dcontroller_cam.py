#!/usr/bin/env python
import rospy
import search
import maze 
import maze_map
import math
import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion

class TurtleBot3:
    
    def __init__(self):
        rospy.init_node('turtlebot3_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        

        self.scaling = 4

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(10.0)

    def get_pose(self):
        while not rospy.is_shutdown():
            try:
                self.transformation = self.tfBuffer.lookup_transform('origin', 'seven', rospy.Time())
                print(self.transformation)
           
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("TF Lookup Exception. Waiting for transformation...")
                self.rate.sleep()
                continue

    def update_pose(self, data):
        self.pose = data

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose[0] / self.scaling - self.transformation.transform.translation.x), 2) +
                    pow((goal_pose[1] / self.scaling - self.transformation.transform.translation.y), 2))

    def linear_vel(self, goal_pose):

        b = 1
        if self.euclidean_distance(goal_pose) > 10:
            k = 0.08
        elif self.euclidean_distance(goal_pose) > 6 and  self.euclidean_distance(goal_pose) <10 :
            k = 0.16
        elif self.euclidean_distance(goal_pose) > 3  and  self.euclidean_distance(goal_pose) <6 :
            k = 0.32
        elif self.euclidean_distance(goal_pose) > 1 and self.euclidean_distance(goal_pose) < 3:
            k = 0.64
        elif self.euclidean_distance(goal_pose) < 1:
            k = 1.2

        if abs(self.angular_vel(goal_pose)) > 0.1:
            b = 0.2
            return  k * self.euclidean_distance(goal_pose) * b


        return  k * self.euclidean_distance(goal_pose) * b

    def angular_vel(self, goal_pose, constant=2):
        
        _, _, current_yaw = euler_from_quaternion([
            self.transformation.transform.rotation.x,
            self.transformation.transform.rotation.y,
            self.transformation.transform.rotation.z,
            self.transformation.transform.rotation.w
        ])
        desired_yaw = atan2(goal_pose[1]/self.scaling  - self.transformation.transform.translation.y,
                            (goal_pose[0]/self.scaling  - self.transformation.transform.translation.x))

        angle: float = (desired_yaw - current_yaw)
    
        pi: float = math.pi
        if angle > pi :
            angle = (-2*pi + angle)
            return constant * angle
        elif angle < -pi :
            angle = (angle + 2 *pi)
            return constant * angle
        else:
            angle = angle
            return constant * angle
        #return constant * angle


    def move2goal(self):
        goal_x = float(input("Set your x goal: "))
        goal_y = float(input("Set your y goal: "))

        distance_tolerance = 0.1 / self.scaling
        vel_msg = Twist()

        # PLANNING TO BE DONE HERE
        current_maze = maze.Maze(1)
        x_initial = self.transformation.transform.translation.x * self.scaling
        y_initial = self.transformation.transform.translation.y * self.scaling
        start_state = [int(x_initial), int(y_initial), 0]
        final_goal = [goal_x, goal_y]
        maze_map.map_1.r1_start = [x_initial, y_initial]
        maze_map.map_1.r1_goal = final_goal

        path = search.aStarSearch(current_maze, 1, start_state, 1, [], [], final_goal)
        print(path)

        for i in range(len(path)):
            if i < (len(path) - 1):
                local_goal = path[i + 1]
                print(local_goal, "its the testing ")
                print("lets go to the next point ")

                while self.euclidean_distance(local_goal) >= distance_tolerance:
                    vel_msg.angular.z = self.angular_vel(local_goal)
                    vel_msg.linear.x = self.linear_vel(local_goal)
                    print(vel_msg)
                    self.velocity_publisher.publish(vel_msg)
                    self.rate.sleep()

                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        don = TurtleBot3()
        don.get_pose()
        don.move2goal()
    except rospy.ROSInterruptException:
        pass
