#!/usr/bin/env python3
import rospy
import search
import maze 
import maze_map
import math
import journal_code_1_orientation
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from math import atan2, floor, sqrt
from tf.transformations import euler_from_quaternion

class TurtleBot3:
    
    def __init__(self):
        rospy.init_node('turtlebot3_controller_0', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
        self.path_publisher = rospy.Publisher('/path_topic_bot_0', Path, queue_size=1)  
        #self.pose_subscriber = rospy.Subscriber('/apriltag_two', Odometry, self.update_pose)  #APRIL TAG     
        self.pose_subscriber = rospy.Subscriber('/tb3_0/odom', Odometry, self.update_pose)          #
        self.path_subscriber =rospy.Subscriber('/path_topic_bot_2', Path, self.update_path2) #OTHER BOT PATH
        self.pose2_subscriber = rospy.Subscriber('/tb3_2/odom', Odometry, self.update_pose2) #OTHER BOT POSE
        
         
        self.pose = Odometry()
        self.pose2 = Odometry()
        self.robot_id=1
        
        self.path=[]
        self.path2=[]
        
        print("path initialised for self is",self.path)
        print("path of other bot subscribed",self.path2)
        self.rate = rospy.Rate(10)
        self.scaling=4
        

    def update_pose(self, data):
        self.pose = data

    def update_path2(self, path_msg):    #OTHER BOT 
        self.path2=[]
        
        for pose_stamped in path_msg.poses:
            x=pose_stamped.pose.position.x
            y=pose_stamped.pose.position.y
            
            self.path2.append([int(x),int(y)])

    def update_pose2(self, data):
        self.pose2 = data
        
        #self.pose2 = data
    def run(self):

        while not rospy.is_shutdown():
            self.rate.sleep()
        
    def euclidean_distance(self, goal_pose):
        
        return sqrt(pow((goal_pose[0]/self.scaling - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose[1]/self.scaling - self.pose.pose.pose.position.y), 2))

    def linear_vel(self, goal_pose):
        
        b = 1
        if self.euclidean_distance(goal_pose) > 10:
            k = 0.04
        elif self.euclidean_distance(goal_pose) > 6  and  self.euclidean_distance(goal_pose) <10 :
            k = 0.08
        elif self.euclidean_distance(goal_pose) > 3  and  self.euclidean_distance(goal_pose) <6 :
            k = 0.16
        elif self.euclidean_distance(goal_pose) > 1 and self.euclidean_distance(goal_pose) < 3:
            k = 0.32
        elif self.euclidean_distance(goal_pose) < 1:
            k = 0.5
        

        
        if abs(self.angular_vel(goal_pose)) > 0.1:
            b = 0.08
            return  k * self.euclidean_distance(goal_pose) * b   #here just give a constant value check k*dist*0.08 value put the same in this 
        


        return  k * self.euclidean_distance(goal_pose) * b #here just give a constant value check k*dist*0.08 value put the same in this

    def angular_vel(self, goal_pose, constant=1):

        _, _, current_yaw = euler_from_quaternion([
            0,
            0,
            self.pose.pose.pose.orientation.z,
            self.pose.pose.pose.orientation.w
        ])
        
        desired_yaw = atan2(goal_pose[1]/self.scaling  - self.pose.pose.pose.position.y,
                            (goal_pose[0]/self.scaling  - self.pose.pose.pose.position.x))

        angle = (desired_yaw - current_yaw)
        

        pi= math.pi
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
        
      

        goal_pose = Odometry()
        goal_x= float(input("Set your x goal: ")) 
        goal_y= float(input("Set your y goal: "))
        goal_tolerance = float(0.2)

        goal_pose.pose.pose.position.x = goal_x/self.scaling 
        goal_pose.pose.pose.position.y = goal_y/self.scaling 
        distance_tolerance = goal_tolerance/self.scaling
        vel_msg = Twist()
        
        #PLANNING TO BE DONE HERE 
        current_maze=maze.Maze(1)

        x_initial =self.pose.pose.pose.position.x*self.scaling
        y_initial =self.pose.pose.pose.position.y*self.scaling
        start_state=[int(round(x_initial)),int(round(y_initial)),0]



        final_goal=[goal_x,goal_y]
        maze_map.map_1.r1_start=[x_initial,y_initial]
        maze_map.map_1.r1_goal = final_goal
        
        
        self.path =search.aStarSearch(current_maze,1,start_state,1,[],[],final_goal)
        
        

        if len(self.path[0]) >= 3:
                del self.path[0][2]
        else:
           self.path[0].append(None)      

        #time.sleep(10)
        # Publish the path
       
        
        
        print(self.path)
        #self.path_length=0
        #self.path_length=range(len(self.path))
        global i
        i=0
        while  i in range(len(self.path)-1):

         i=i+1
         if i < (len(self.path)):
            local_goal=self.path[i]
            
            print("lets go to next point ",local_goal)
        
            while self.euclidean_distance(local_goal) >= distance_tolerance:
                #here it need to subscribe nodes :
                path_msg = Path()
               
                for point in self.path:
                    
                    pose_stamped = PoseStamped()
                    pose_stamped.pose.position.x = point[0]
                    pose_stamped.pose.position.y = point[1]
                    path_msg.poses.append(pose_stamped)


                self.path_publisher.publish(path_msg)
                   
               
                #print("self bot is publishing the path =",path_msg)
               
              
                  
                
                #print("collision testing ...")
                #taking  intersection
                collision_index =[index for index, (item1, item2) in enumerate(zip(self.path,self.path2)) if item1 == item2 and self.path.count(item1) == 1 and self.path2.count(item2) == 1]
                
            
                if collision_index:

                    print("collision found ")
                    q = collision_index[0] #q is the index of the coordinate where collision is going to happen
                    
                    #calculating both bots orientation at collision point , to determine left/right side w.r.t bot
                    r1_ang = journal_code_1_orientation.track_orientation([self.path[q-1],self.path[q]]) 
                    r2_ang = journal_code_1_orientation.track_orientation([self.path2[q-1],self.path2[q]])
                    print("me", r1_ang)
                    print("other", r2_ang)
                
                    if journal_code_1_orientation.right(r1_ang[0], r2_ang[0]) == True:
                        
                        print("other bot is right side ")
                        current_maze=maze.Maze(1)
                        
                        collison_point = self.path[q]
                        print("collision point is",collison_point)
                        positions_after_collision=self.path[q+1:] 
                       
                        #p2=[robot.current_state[0],robot.current_state[1]] #other robot position subscribe to be done here coordinates to be stored 
                        
                        #floor & round can be used below bots but modified path changes    
                        current_state_of_robot = [round(self.pose.pose.pose.position.x*self.scaling),round(self.pose.pose.pose.position.y*self.scaling)] # SELF BOT positions
                        #print("im at position",current_state_of_robot)
                        #neighbour_robot = [round(self.pose2.pose.pose.position.x*self.scaling),round(self.pose2.pose.pose.position.y*self.scaling)] #OTHER BOT  positions
                        neighbour_robot=self.path2[q-1]
                        #neighbour_robot=self.path0[q-1]
                        #print("other bot is at ",neighbour_robot)
                        #Replanning here 
                        self.path = search.aStarSearch(current_maze, 1, current_state_of_robot, 2, collison_point, neighbour_robot, positions_after_collision)
                        print("new path  ",self.path)

                        local_goal=self.path[1]
                        #print("local goal changed to ",local_goal)
                        
                        i = 0
                        
                        #set new v,w
                        vel_msg.angular.z = self.angular_vel(local_goal)
                        vel_msg.linear.x = self.linear_vel(local_goal)
                        
                        self.velocity_publisher.publish(vel_msg)
                        self.rate.sleep()
        
                    else:
                        #set v,w
                   
                        vel_msg.angular.z = self.angular_vel(local_goal)
                        vel_msg.linear.x = self.linear_vel(local_goal)
                        print("collision detected but i will not follow rule ")
                        self.velocity_publisher.publish(vel_msg)

                        self.rate.sleep()
                else:
                    #set v.w
          
                    print("collision not detected")
                    print("i have to go",local_goal)
                    present_position =[round(self.pose.pose.pose.position.x*self.scaling),round(self.pose.pose.pose.position.y*self.scaling)]
                    print("i am present at scaling location :",present_position)
                
                    vel_msg.angular.z = self.angular_vel(local_goal)
                    vel_msg.linear.x = self.linear_vel(local_goal)
                    
                    #print("my angular velocity is ",vel_msg.angular.z)
                    #print("my linear velocity is ",vel_msg.linear.x)
                    self.velocity_publisher.publish(vel_msg)
                    #time.sleep(2)
                    self.rate.sleep()
                    
            print("self bot subscribing the the path=",self.path2)  #OTHER BOT        
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            rospy.spin
if __name__ == '__main__':
    try:
        eshant= TurtleBot3()
        eshant.move2goal()
    except rospy.ROSInterruptException:
        pass