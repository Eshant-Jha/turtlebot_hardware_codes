#!/usr/bin/env python3
from re import T
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

from std_msgs.msg import Bool

class TurtleBot3:
    
    def __init__(self):
        rospy.init_node('turtlebot3_controller_0', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
        self.path_publisher = rospy.Publisher('/path_topic_bot_0', Path, queue_size=1)  
        #self.pose_subscriber = rospy.Subscriber('/apriltag_zero', Odometry, self.update_pose)  #APRIL TAG     
        self.pose_subscriber = rospy.Subscriber('/tb3_0/odom', Odometry, self.update_pose)     

        self.path1_subscriber =rospy.Subscriber('/path_topic_bot_1', Path, self.update_path1) #OTHER BOT PATH
        self.pose1_subscriber = rospy.Subscriber('/tb3_1/odom', Odometry, self.update_pose1) #OTHER BOT POSE
        #self.pose1_subscriber = rospy.Subscriber('/apriltag_one', Odometry, self.update_pose1)    

        self.path2_subscriber =rospy.Subscriber('/path_topic_bot_2', Path, self.update_path2) #OTHER BOT PATH
        self.pose2_subscriber = rospy.Subscriber('/tb3_2/odom', Odometry, self.update_pose2) #OTHER BOT POSE        
        #self.pose2_subscriber = rospy.Subscriber('/apriltag_two', Odometry, self.update_pose2)

        
        


        #+=+=+=+=+=+=+=+=+=+=+=+=+=+=+==+=+=+=+=+=

        self.sync1_subscriber = rospy.Subscriber('bot_sync_1', Bool, self.update_bot1_finished)
        self.sync2_subscriber = rospy.Subscriber('bot_sync_2', Bool, self.update_bot2_finished)

        self.sync0_pub = rospy.Publisher('bot_sync_0', Bool, queue_size=10)

        self.bot0_finished = Bool()
        self.bot1_finished = Bool()
        self.bot2_finished = Bool()
        #+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=++=+=+=+=+=+=


            
        self.pose = Odometry()
        self.pose1 = Odometry()
        self.pose2 = Odometry()

        self.robot_id=1
        
        self.path=[]
        self.path1=[]
        self.path2=[]
        
        #print("path initialised for self is",self.path)
        #print("path of other bot subscribed",self.path2)
        self.rate = rospy.Rate(100)
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
    
    def update_path1(self, path_msg):    #OTHER BOT 
        self.path1=[]
        
        for pose_stamped in path_msg.poses:
            x=pose_stamped.pose.position.x
            y=pose_stamped.pose.position.y
            
            self.path1.append([int(x),int(y)])

    def update_pose1(self, data):
        self.pose1 = data

    #+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=++=+=+=+=+=+=
        
    def update_bot1_finished(self, data):
        self.bot1_finished = data

    def update_bot2_finished(self, data):
        self.bot2_finished = data


    #+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=++=+=+=+=+=+=



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

        
        print(self.path)

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


                #+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=++=+=+=+=+=+=
                self.bot0_finished.data = False

                self.sync0_pub.publish(self.bot0_finished)
                #+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=++=+=+=+=+=+=

            
               
                for point in self.path:
                    
                    pose_stamped = PoseStamped()
                    pose_stamped.pose.position.x = point[0]
                    pose_stamped.pose.position.y = point[1]
                    path_msg.poses.append(pose_stamped)


                self.path_publisher.publish(path_msg)
                   

                collision_index_1 =[index for index, (item1, item2) in enumerate(zip(self.path,self.path1)) if item1 == item2 and self.path.count(item1) == 1 and self.path1.count(item2) == 1]
                collision_index_2 =[index for index, (item1, item2) in enumerate(zip(self.path,self.path2)) if item1 == item2 and self.path.count(item1) == 1 and self.path2.count(item2) == 1]
                
                ###################################################### 
                bot_2_x = self.pose2.pose.pose.position.x
                bot_2_y = self.pose2.pose.pose.position.y 
                bot_2_position =[bot_2_x*self.scaling,bot_2_y*self.scaling]
                #print("bot 0 is at  ",bot_2_position)
                threshold_distance_2= self.euclidean_distance(bot_2_position)*self.scaling

                bot_1_x = self.pose1.pose.pose.position.x
                bot_1_y = self.pose1.pose.pose.position.y 
                bot_1_position =[bot_1_x*self.scaling,bot_1_y*self.scaling]
                #print("bot 1 is at  ",bot_1_position)
                threshold_distance_1= self.euclidean_distance(bot_1_position)*self.scaling


                ######################################################



                if  collision_index_2 and threshold_distance_2 <=3:
                    print("oh shit collision found with bot 2 ")

                    q = collision_index_2[0]
                    print("oh shit collision found with bot 2 ", q)
                    r2_ang = journal_code_1_orientation.track_orientation([self.path2[q-1],self.path2[q]])
                    r1_ang = journal_code_1_orientation.track_orientation([self.path[q-1],self.path[q]])
                        
                    if journal_code_1_orientation.right(r1_ang[0], r2_ang[0]) == True:
                        print("my bad  bot2 is right side ")
                        current_maze=maze.Maze(1)
                        
                        collison_point = self.path[q]
                        print("collision point is",collison_point)
                        positions_after_collision=self.path[q+1:] 

                        current_state_of_robot = [round(self.pose.pose.pose.position.x*self.scaling),round(self.pose.pose.pose.position.y*self.scaling)]

                        neighbour_robot=self.path2[q-1]

                        self.path = search.aStarSearch(current_maze, 1, current_state_of_robot, 2, collison_point, neighbour_robot, positions_after_collision)
                        print("new path  ",self.path)

                        local_goal=self.path[1]

                        
                        i = 0
                        vel_msg.angular.z = self.angular_vel(local_goal)
                        vel_msg.linear.x = self.linear_vel(local_goal)
                        self.velocity_publisher.publish(vel_msg)
                        self.rate.sleep()



                    elif journal_code_1_orientation.right(r1_ang[0], r2_ang[0]) == 2:

                        #print("other bot is head-on ")
                        current_maze=maze.Maze(1)
                        collison_point = self.path[q]
                        positions_after_collision=self.path[q+1:] 
                        current_state_of_robot = self.path[q-1]    #this is necessary to have logic work \
                        #shortcoming is that from present state it has to directly go to q-1 BUT IT CAN BE RECTIFIED BY USING THRESHOLD DISTANCE 
                        neighbour_robot=self.path2[q-1]
                        self.path = search.aStarSearch(current_maze, 1, current_state_of_robot, 3, collison_point, neighbour_robot, positions_after_collision)
                        #print("new path  ",self.path)
                        #print("self bot subscribing the the path=",self.path0)    
                        local_goal=self.path[1]
                        #print("local goal changed to ",local_goal)
                        i = 0 #SINCE PATH REPLANNED SO IT HAS TO START FROM BEGINNING 
                        vel_msg.angular.z = self.angular_vel(local_goal)
                        vel_msg.linear.x = self.linear_vel(local_goal)
                        self.velocity_publisher.publish(vel_msg)
                        self.rate.sleep()



                    else:
                        print("i dont give a shit bot2 ")
                        vel_msg.angular.z = self.angular_vel(local_goal)
                        vel_msg.linear.x = self.linear_vel(local_goal)
                        self.velocity_publisher.publish(vel_msg)
                        self.rate.sleep()

    


                elif collision_index_1 and threshold_distance_1 <=3: 
                    
                    q = collision_index_1[0]
                    print("oh shit collision found with bot 1 ",q)
                    r2_ang = journal_code_1_orientation.track_orientation([self.path1[q-1],self.path1[q]])
                    r1_ang = journal_code_1_orientation.track_orientation([self.path[q-1],self.path[q]])

                    if journal_code_1_orientation.right(r1_ang[0], r2_ang[0]) == True:
                            
                        print("my bad   bot1 is right side ")
                        current_maze=maze.Maze(1)

                        collison_point = self.path[q]
                        print("collision point is",collison_point)
                        positions_after_collision=self.path[q+1:] 

                        current_state_of_robot = [round(self.pose.pose.pose.position.x*self.scaling),round(self.pose.pose.pose.position.y*self.scaling)] # SELF BOT positions

                        neighbour_robot=self.path1[q-1]

                        self.path = search.aStarSearch(current_maze, 1, current_state_of_robot, 2, collison_point, neighbour_robot, positions_after_collision)
                        print("new path  ",self.path)

                        local_goal=self.path[1]

                        
                        i = 0
                        vel_msg.angular.z = self.angular_vel(local_goal)
                        vel_msg.linear.x = self.linear_vel(local_goal)
                        self.velocity_publisher.publish(vel_msg)
                        self.rate.sleep()



                    elif journal_code_1_orientation.right(r1_ang[0], r2_ang[0]) == 2:

                        #print("other bot is head-on ")
                        current_maze=maze.Maze(1)
                        collison_point = self.path[q]
                        positions_after_collision=self.path[q+1:] 
                        current_state_of_robot = self.path[q-1]    #this is necessary to have logic work \
                        #shortcoming is that from present state it has to directly go to q-1 BUT IT CAN BE RECTIFIED BY USING THRESHOLD DISTANCE 
                        neighbour_robot=self.path1[q-1]
                        self.path = search.aStarSearch(current_maze, 1, current_state_of_robot, 3, collison_point, neighbour_robot, positions_after_collision)
                        #print("new path  ",self.path)
                        #print("self bot subscribing the the path=",self.path0)    
                        local_goal=self.path[1]
                        #print("local goal changed to ",local_goal)
                        i = 0 #SINCE PATH REPLANNED SO IT HAS TO START FROM BEGINNING 
                        vel_msg.angular.z = self.angular_vel(local_goal)
                        vel_msg.linear.x = self.linear_vel(local_goal)
                        self.velocity_publisher.publish(vel_msg)
                        self.rate.sleep()



                    else:
                        print("i dont give a shit bot1 ")    
                        vel_msg.angular.z = self.angular_vel(local_goal)
                        vel_msg.linear.x = self.linear_vel(local_goal)
                        self.velocity_publisher.publish(vel_msg)
                        self.rate.sleep()



                else:
          
                    #print("collision not detected")
                    #print("i have to go",local_goal)
                    present_position =[round(self.pose.pose.pose.position.x*self.scaling),round(self.pose.pose.pose.position.y*self.scaling)]
                    #print("i am present at  :",present_position)
                
                    vel_msg.angular.z = self.angular_vel(local_goal)
                    vel_msg.linear.x = self.linear_vel(local_goal)

                    self.velocity_publisher.publish(vel_msg)
                    #time.sleep(2)
                    self.rate.sleep()

                    
            #print("bot1 path=",self.path1)  #OTHER BOT  
            #print("bot2 path=",self.path2)      
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            


            #+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=++=+=+=+=+=+=
            
            self.bot0_finished.data = True
            self.sync0_pub.publish(self.bot0_finished)

            time.sleep(0.1)

            while self.bot1_finished.data == False or self.bot0_finished.data == False or self.bot2_finished.data == False:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                if self.bot1_finished.data == True and self.bot0_finished.data == True and self.bot2_finished.data == True:
                    break

            time.sleep(0.2)

            #+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=++=+=+=+=+=+=

            


            rospy.spin
if __name__ == '__main__':
    try:
        eshant= TurtleBot3()
        eshant.move2goal()
    except rospy.ROSInterruptException:
        pass
