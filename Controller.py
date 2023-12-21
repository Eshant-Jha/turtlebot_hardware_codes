#!/usr/bin/env python
import rospy
import search
import maze 
import maze_map

from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import Odometry
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion
from apriltag_ros.msg import AprilTagDetectionArray
class TurtleBot3:
    
    def __init__(self):
        rospy.init_node('turtlebot3_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        #self.pose_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray(), self.update_pose)
        self.pose = Odometry()
        #self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.scaling=2
    def update_pose(self, data):
        self.pose = data

    def euclidean_distance(self, goal_pose):
        
        return sqrt(pow((goal_pose[0]/self.scaling - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose[1]/self.scaling - self.pose.pose.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=0.04):
        return constant * self.euclidean_distance(goal_pose)

    def angular_vel(self, goal_pose, constant=0.03):
        
        _, _, current_yaw = euler_from_quaternion([
            self.pose.pose.pose.orientation.x,
            self.pose.pose.pose.orientation.y,
            self.pose.pose.pose.orientation.z,
            self.pose.pose.pose.orientation.w
        ])
        desired_yaw = atan2(goal_pose[1]/self.scaling - self.pose.pose.pose.position.y,
                            goal_pose[0]/self.scaling - self.pose.pose.pose.position.x)
        return constant * (desired_yaw - current_yaw)

    def move2goal(self):
        
      

        goal_pose = Odometry()
        goal_x=float(input("Set your x goal: ")) 
        goal_y=float(input("Set your y goal: "))
        goal_tolerance = float(input("Set your tolerance: "))

        goal_pose.pose.pose.position.x = goal_x/self.scaling
        goal_pose.pose.pose.position.y = goal_y/self.scaling
        distance_tolerance = goal_tolerance/self.scaling
        vel_msg = Twist()
        
        #PLANNING TO BE DONE HERE 
        current_maze=maze.Maze(1)

        x_initial =self.pose.pose.pose.position.x*self.scaling
        y_initial =self.pose.pose.pose.position.y*self.scaling
        start_state=[int(x_initial),int(y_initial),0]
        print(start_state, "testing line 59 ")
        #start_state=[5,1,0]
        print(start_state, "testing line 61 ")

        final_goal=[goal_x,goal_y]
        maze_map.map_1.r1_start=[x_initial,y_initial]
        maze_map.map_1.r1_goal = final_goal
        
        
        path =search.aStarSearch(current_maze,1,start_state,1,[],[],final_goal)
        print(path)

        for i in range(len(path)):
         if i < (len(path)-1):
            local_goal=path[i+1]
            print(local_goal,"its the testing ")
            print("lets go to next point ")
        
            while self.euclidean_distance(local_goal) >= distance_tolerance:
                vel_msg.angular.z = self.angular_vel(local_goal)
                vel_msg.linear.x = self.linear_vel(local_goal)
                
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        don= TurtleBot3()
        don.move2goal()
    except rospy.ROSInterruptException:
        pass