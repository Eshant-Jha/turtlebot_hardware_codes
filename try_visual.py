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
import matplotlib.transforms as transforms #for error ellipse transforms
import matplotlib.pyplot as plt #for plotting
from matplotlib.patches import Ellipse #for plotting Ellipse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse
import numpy as np
import os
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Visual:
    fig, ax1 = plt.subplots()
    colormap = {0: 'red', 1: 'blue', 2: 'green'}
    heading_color_map = {0: 'red', 1: 'blue', 2: 'green'}

    def __init__(self, bot_id):
        self.bot_id = bot_id
        self.robot_pose = [0.0, 0.0, 0.0]
        self.scale_x = self.scale_y = 100
        self.x_offset = self.y_offset = 1.500
        self.heading_line_length = 10.0

        # Initialize plot setup
        file_path = os.path.join(cwd_path, 'imag.jpeg')
        img = plt.imread(file_path)
        Visual.ax1.imshow(img)
        Visual.ax1.set(xlabel='X(x 10mm)', ylabel='Y(x 10mm)')

                

        # Plot handles
        self.robot_point_plot_handle = Visual.ax1.scatter(0, 0, alpha=1.0, s=50, color=Visual.colormap[bot_id])
        self.heading_color = Visual.heading_color_map.get(bot_id, 'black')  # Set the color for the heading line
        self.heading_line_plot_handle, = Visual.ax1.plot([], [], color=self.heading_color, linestyle='-', linewidth=2)
        #self.heading_line_plot_handle, = Visual.ax1.plot([], [], 'g-')
        self.error_ellipse_plot = Ellipse((0, 0), width=1.0, height=1.0, angle=0.0, edgecolor='y', fc='None', lw=2)
        Visual.ax1.add_patch(self.error_ellipse_plot)

        # Path trail initialization
        self.path_x = []
        self.path_y = []
        self.path_line_handle, = Visual.ax1.plot([], [], color=Visual.colormap[bot_id], lw=2)

    def update_plot(self, frame):
        # Update robot position plot
        robot_x_plot = (self.robot_pose[0] * self.scale_x) + self.x_offset     
        robot_y_plot = (self.robot_pose[1] * self.scale_y) + self.y_offset
        self.robot_point_plot_handle.set_offsets([robot_x_plot, robot_y_plot])   

        # Update heading plot
        heading_x_plot = [robot_x_plot, robot_x_plot + self.heading_line_length * np.cos(np.pi/2 -self.robot_pose[2])]
        heading_y_plot = [robot_y_plot, robot_y_plot + self.heading_line_length * np.sin(np.pi/2 -self.robot_pose[2])]
        self.heading_line_plot_handle.set_data(heading_x_plot, heading_y_plot)

        # Update error ellipse
        error_x, error_y = 0.01, 0.01  # Replace with actual error values
        self.error_ellipse_plot.set_center((robot_x_plot, robot_y_plot))
        self.error_ellipse_plot.width = error_x / self.scale_x
        self.error_ellipse_plot.height = error_y / self.scale_y
        self.error_ellipse_plot.angle = np.rad2deg(self.robot_pose[2]) 

        # Update path trail
        self.path_x.append(robot_x_plot)
        self.path_y.append(robot_y_plot)
        self.path_line_handle.set_data(self.path_x, self.path_y)

    def ekf_callback(self, data):
        self.robot_pose[0] = data.pose.pose.position.y
        self.robot_pose[1] = data.pose.pose.position.x
        quat = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        )
        _, _, self.robot_pose[2] = euler_from_quaternion(quat)
        
        

if __name__ == '__main__':
    rospy.init_node('robots_visualisation')
    cwd_path = os.path.dirname(os.path.abspath(__file__))

    robot1 = Visual(0)
    robot2 = Visual(1)
    robot3 = Visual(2)

    rospy.Subscriber('/tb3_0/odom', Odometry, lambda data: robot1.ekf_callback(data))
    rospy.Subscriber('/tb3_1/odom', Odometry, lambda data: robot2.ekf_callback(data))
    rospy.Subscriber('/tb3_2/odom', Odometry, lambda data: robot3.ekf_callback(data))

    ani1 = FuncAnimation(Visual.fig, robot1.update_plot, interval=100)
    ani2 = FuncAnimation(Visual.fig, robot2.update_plot, interval=100)
    ani3 = FuncAnimation(Visual.fig, robot3.update_plot, interval=100)

    plt.show()
    rospy.spin()
