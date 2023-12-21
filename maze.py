# -*- coding: utf-8 -*-
"""
Created on Tue Jun 20 21:26:13 2023

@author: OE22S300
"""
import matplotlib
import matplotlib.pyplot as plt
import copy

import maze_map
 

class Maze:
  """
  This class outlines the structure of the maze problem
  """
  
  maze_map = []# To store map data, start and goal points
  
  # [delta_x, delta_y, description]
  five_neighbor_actions = {'up':[-1, 0], 'down':[1, 0], 'left': [0, -1], 'right': [0, 1] } #'stop': [0, 0]}
    
 
      
  # default constructor
  def __init__(self, id):
      """
      Sets the map as defined in file maze_maps
      """
      #Set up the map to be used
      self.maze_map = maze_map.maps_dictionary[id]  
      
      return
     
  def getStartState(self, robot_id):
     """
     Returns the start state for the search problem 
     """
     if robot_id==1:
      start_state = self.maze_map.r1_start 
     elif robot_id==2:
      start_state= self.maze_map.r2_start
     else:
       start_state= self.maze_map.r3_start  
     return start_state
 
  def getGoalState(self,robot_id):
     """
     Returns the start state for the search problem 
     """
     if robot_id==1:
         goal_state = self.maze_map.r1_goal
     elif robot_id==2:
         goal_state= self.maze_map.r2_goal
     else:
         goal_state =self.maze_map.r3_goal
         
     return goal_state
    
  def isGoalState(self, robot_id, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     if state[0:2] == self.getGoalState(robot_id):
         return True
     else:
         return False

  def isObstacle(self, state):
      """
        state: Search state
     
      Returns True if and only if the state is an obstacle
     
      """

      if self.maze_map.map_data[state[0]][state[1]] == maze_map.obstacle_id:
          return True
      else:
          return False
    
  

#Check the p3 point position w.r.t to line joining p1 to p2
  def point_position_with_line(self,p1, p2, p3):
      # p1 is the collision point
      # p2 is a point on the line joining collision and other bot coordinates (P1P2)
      # p3 is the state in get_successor

      x1, y1 = p1
      x2, y2 = p2
      x3, y3 = p3

      # Calculate vectors representing the line (P1P2) and the point (P1P3)
      line_vector = (x2 - x1, y2 - y1)
      point_vector = (x3 - x1, y3 - y1)

      # Calculate the cross product of the two vectors
      cross_product = line_vector[0] * point_vector[1] - line_vector[1] * point_vector[0]
      return cross_product>=0   #will return only when point is on left
  
    
  def getSuccessors(self, state): #this is used in global planning 
 
     """
       state: Seacrh state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     successors = []  
     for action in self.five_neighbor_actions:
         
         #Get individual action
         del_x, del_y = self.five_neighbor_actions.get(action) 
        
         #Get successor
         new_successor = [state[0] + del_x , state[1] + del_y]   
         new_action = action
         
         # Check for static obstacle 
         
         if self.isObstacle(new_successor):
             continue
      
         #cost
         new_cost = maze_map.free_space_cost         
         successors.append([new_successor, new_action, new_cost])
         
     return successors
 
    
  def getSuccessors_local(self, robot_id, state, collison_point, neighbor_robot_point, start_state): #used for local planning 
     """
       state: Seacrh state
      
    to be added in this code FOR HEAD on COLLISION BOTH REPLANS CONSIDERING VECTOR FROM CURRENT TO COLLISON AND TAKING LEFT SIDE SUCCESSOR INVALID
     """
     #print("local planning doing for ",robot_id)
     successors = []  
     for action in self.five_neighbor_actions:
         
         #Get individual action
         del_x, del_y = self.five_neighbor_actions.get(action) 
            
         #Get successor
         new_successor = [state[0] + del_x , state[1] + del_y]    #new_successor = [state[0] + del_x , state[1] + del_y, state[2]+1]
         new_action = action
         
         #Check for static obstacle 
         if self.isObstacle(new_successor):
             continue
         
         #COLREGS rule 15 applied for collision avoidance with other bot /here its making obstacle zone 

         if  self.point_position_with_line( start_state,neighbor_robot_point, new_successor) and \
               self.point_position_with_line(neighbor_robot_point,collison_point ,new_successor):
               continue
            
         new_cost = maze_map.free_space_cost  
         #print(new_successor)            
         successors.append([new_successor, new_action, new_cost])
         
     return successors
    
      
      
      
      
      
      
