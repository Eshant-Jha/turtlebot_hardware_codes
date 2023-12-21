#!/usr/bin/env python
from distutils.util import run_2to3
import math

"orientation tracking of the bot "


def track_orientation(points):
    orientations = []

    for i in range(1, len(points)):
        x1, y1 = points[i-1]
        x2, y2 = points[i]

        dx = x2 - x1
        dy = y2 - y1

        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)

        # Convert the angle to be within the range [0, 360)
        if angle_deg < 0:
            angle_deg += 360

        orientations.append(angle_deg)

    return orientations

# Example usage
#path = [[6, 4], [6, 5], [6, 6], [6, 7], [6, 8], [5, 8], [4, 8], [3, 8], [2, 8], [1, 8], [1, 9]]

def equalise_path(r_path1, r_path2, r_path3):
    len_path1, len_path2, len_path3 = len(r_path1), len(r_path2), len(r_path3)
    max_length = max(len_path1, len_path2, len_path3)
    
    diff_path1 = max_length - len_path1
    diff_path2 = max_length - len_path2
    diff_path3 = max_length - len_path3

    r_path1 += [r_path1[-1]] * diff_path1 if diff_path1 > 0 else []
    r_path2 += [r_path2[-1]] * diff_path2 if diff_path2 > 0 else []
    r_path3 += [r_path3[-1]] * diff_path3 if diff_path3 > 0 else []

    return r_path1, r_path2, r_path3





def calculate_angle(bot1_x, bot1_y, bot1_orientation, bot2_x, bot2_y, bot2_orientation):
    dx = bot2_x - bot1_x #
    dy = bot2_y - bot1_y

    # Calculate the angle between the two bots using their orientations
    angle_rad = math.atan2(dy, dx) - math.radians(bot1_orientation)
    angle_deg = math.degrees(angle_rad)

    # Convert the angle to be within the range [0, 360)
    if angle_deg < 0:
        angle_deg += 360
   
    if angle_deg in range(90,270):
      print("left hai") 
      angle_deg=0
  

    else: 
      angle_deg=1
  
    
    return angle_deg


def right(r1_ang,r2_ang): 
    angle_difference = r2_ang - r1_ang # due to one of the four use cases i edit this portion by DON on 21/12/23
    if angle_difference > -180 and angle_difference <= 0:
        return False
    elif angle_difference > 0 and angle_difference < 180:
        return True  
    elif r1_ang == 270 and r2_ang == 0:
        return True

 









 

