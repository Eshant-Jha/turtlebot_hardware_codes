#!/usr/bin/env python3
import os    # python library for system file path
import numpy as np  # the numerical python library
import rospy #python library for ros
from geometry_msgs.msg import Twist  #importing the messgae for publishing
#from geometry_msgs.msg import PoseWithCovarianceStamped #importig message for publish
from tf.transformations  import euler_from_quaternion # feature necesssary for converting the quaternion to euler angles
from nav_msgs.msg import Odometry
import matplotlib #for plotting
import matplotlib.transforms as transforms #for error ellipse transforms
import matplotlib.pyplot as plt #for plotting
from matplotlib.patches import Ellipse #for plotting Ellipse

plt.ion()

class Visual :

    fig, ax1 = plt.subplots()
    colormap={0:'red',1:'blue',2:'green'}

    def __init__(self,bot_id) :
          
        self.bot_id= bot_id
        self.scaling = 4 
        self.robot_pose = [0.0, 0.0, 0.0] 
        self.scale_x ,self.scale_y =[4,4]

        self.x_offset = 1.500  # x offset to adjust with the map, home coordinates
        self.y_offset = 1.500 # y offset to adjust with the map, home coordinates
        self.scale_x =  self.scale_y =  100     # scale for x values to adjust with the map
          # scale for y values to adjust with the map
        self.heading_line_length = 15.0 #length of the black geading line

        
        ##########################

        
        # Plot map as a background image
        file_path = os.path.join(cwd_path, 'imag.jpeg')
        img = plt.imread(file_path)
        Visual.ax1.imshow(img)  # Use the common axes

        # Figure setup
        Visual.ax1.set(xlabel='X(x 10mm)', ylabel='Y(x 10mm)')

        # Initialize scatter plot handle
        #self.robot_point_plot_handle = Visual.ax1.scatter(0, 0, alpha=1.0, s=50, color='green')

        # Initialize error ellipse plot handle
        self.error_ellipse_plot = Ellipse((0, 0), width=1.0, height=1.0, angle=0.0, edgecolor='y', fc='None', lw=2)
        Visual.ax1.add_patch(self.error_ellipse_plot)


        #Plot robot
        robot_x_plot = (self.robot_pose[0]*self.scale_x) + self.x_offset
        print("asdfg",robot_x_plot)
        robot_y_plot = (self.robot_pose[1]*self.scale_y) + self.y_offset

       

        if self.bot_id ==0 :
           colour ='red'
           edgecolour='r'
        elif self.bot_id ==1  :
            colour= 'blue'
            edgecolour = 'b'
        else:
            colour ='green'   
            edgecolour = 'g' 


        self.robot_point_plot_handle = Visual.ax1.scatter(robot_x_plot, robot_y_plot, alpha=1.0, s=50, color=colour )   # plotting the robot as a point

        #Plot trail
        self.prev_robot_x_plot = 0 # robot_x_plot
        self.prev_robot_y_plot = 0 #robot_y_plot
        #Visual.ax1.plot([robot_x_plot, self.prev_robot_x_plot], [robot_y_plot, self.prev_robot_y_plot],'k--')

        #Plot heading
        heading_x_plot = [robot_x_plot, robot_x_plot + self.heading_line_length*np.cos(self.robot_pose[2])]
        heading_y_plot = [robot_y_plot, robot_y_plot + self.heading_line_length*np.sin(self.robot_pose[2])]
        self.heading_line_plot_handle, = Visual.ax1.plot(heading_x_plot, heading_y_plot,'g-')

        self.heading_line_plot_handle.set_xdata(heading_x_plot)
        self.heading_line_plot_handle.set_ydata(heading_y_plot)

        #Plot error ellipse
        self.error_ellipse_plot = Ellipse(xy = (robot_x_plot, robot_y_plot), width = 1.0, height = 1.0, angle = 0.0, edgecolor=edgecolour, fc='None', lw=2)
        Visual.ax1.add_patch(self.error_ellipse_plot)

        #Set legend
        Visual.ax1.legend()



  
    def ekf_callback(self ,data):

        """A callback funcction which plots the updated ekf position and error ellipse"""

        #Update the robot pose from message
        self.robot_pose[0] = data.pose.pose.position.x
        self.robot_pose[1] = data.pose.pose.position.y
        print("bot id ",self.bot_id)
        
        #Converting the quaternion to euler angle
        measurement_quat = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        _,_,self.robot_pose[2] = euler_from_quaternion(measurement_quat)
        

        #Update robot position plot

        robot_x_plot = (self.robot_pose[1]*self.scale_x) + self.x_offset
        robot_y_plot = (self.robot_pose[0]*self.scale_y) + self.y_offset
        self.robot_point_plot_handle.set_offsets([robot_x_plot, robot_y_plot])  


        if self.prev_robot_x_plot==0:
            self.prev_robot_x_plot=robot_x_plot
            self.prev_robot_y_plot=robot_y_plot
    
        #Update robot trail
        trail_color=Visual.colormap.get(self.bot_id,'black')   
        Visual.ax1.plot([robot_x_plot, self.prev_robot_x_plot], [robot_y_plot, self.prev_robot_y_plot],color=trail_color ,linestyle='--')
        self.prev_robot_x_plot = robot_x_plot
        self.prev_robot_y_plot = robot_y_plot


        #Update heading plot
        heading_x_plot = [robot_x_plot, robot_x_plot + self.heading_line_length*np.sin(self.robot_pose[2])]
        heading_y_plot = [robot_y_plot, robot_y_plot + self.heading_line_length*np.cos(self.robot_pose[2])]
        
        self.heading_line_plot_handle.set_xdata(heading_x_plot)
        self.heading_line_plot_handle.set_ydata(heading_y_plot)
         
        #############################################################################
        ###error was used when ekf waqs in plot , right now error values given randon number to statisfy the below code lines 
        error_angle = 0.01
        
                   #np.arctan2(eigenvects[1,0], eigenvects[0,0])   
        error_x = 0.01               #np.sqrt(eigenvals[0])
        error_y = 0.01               #np.sqrt(eigenvals[1])

        #########################################################################

        if self.error_ellipse_plot is not None :           ##   gpt
            self.error_ellipse_plot.remove()               ##   gpt

        #plotting the extracted Ellipse
        self.error_ellipse_plot = Ellipse(xy = (robot_x_plot, robot_y_plot), width = error_x/self.scale_x, height = error_y/self.scale_y, angle = np.rad2deg(self.robot_pose[2] + error_angle), edgecolor='y', fc='None', lw=2)
        Visual.ax1.add_patch(self.error_ellipse_plot)


if __name__ == '__main__':

    #intialising a node for the vizualisation part
    rospy.init_node('robots_visualisation')
    cwd_path = os.path.dirname(os.path.abspath(__file__)) #gettting the parent directory path
    #Plot map as a backgorund image
    file_path = os.path.join(cwd_path, 'imag.jpeg')       #finding full file path         #save the image file in same folder 
    img = plt.imread(file_path)                           #reading the background image

    #Multi-bots object creation 
   
    robot1=Visual(0)
    robot2=Visual(1)
    robot3=Visual(2)


    #subscribing the required topic and updating its callback function
 

    
    rospy.Subscriber('/tb3_0/odom', Odometry, lambda data: robot1.ekf_callback(data))
    rospy.Subscriber('/tb3_1/odom', Odometry, lambda data: robot2.ekf_callback(data))
    rospy.Subscriber('/tb3_2/odom', Odometry, lambda data: robot3.ekf_callback(data))  


    # rospy.Subscriber('/apriltag_zero', Odometry, lambda data: robot1.ekf_callback(data))
    # rospy.Subscriber('/apriltag_one', Odometry, lambda data: robot2.ekf_callback(data))
    # rospy.Subscriber('/apriltag_two', Odometry, lambda data: robot3.ekf_callback(data))
    
    #rospy.spin()
    #rate
    rate = rospy.Rate(400)     #4Hz
    plt.show(block=True)
    rate.sleep()
