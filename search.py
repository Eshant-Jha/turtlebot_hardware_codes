# -*- coding: utf-8 -*-
"""
Created on Tue Jun 20 20:31:41 2023

@author: OE22S300
"""

import operator

heusritic_weight =  1.0

def heuristic_1(problem, state, heusritic_weight, robot_id):
    """
    Manhattan distance
    """
    goal = problem.getGoalState(robot_id)
    goal = problem.getGoalState(robot_id)
    del_x_1 = abs(state[0] - goal[0])
    del_y_1 = abs(state[1] - goal[1])
    
    return heusritic_weight*(del_x_1 + del_y_1 )


def aStarSearch(problem, robot_id, start_state, planner, collison_point, neighbor_robot_point, goal_states):
  "Search the node that has the lowest combined cost and weighted heuristic first."
  #Create explored list to store popped nodes
  explored = []
  #Create Fringe to store all nodes to be expaned
  fringe = []
  #Add the start state to Fringe 
  fringe.append([start_state, [start_state], 0, (heuristic_1(problem, start_state, heusritic_weight, robot_id))])
 

  print("Planning...")  
  
  while len(fringe)>0:
      
      fringe = sorted(fringe, key = operator.itemgetter(3))
      
      #Pop least cost node and add to explored list
      current_node = fringe.pop(0)
  
      explored.append(current_node[0]) # only the state needs to be added to explored list 
     
      if planner==1:
          if problem.isGoalState(robot_id ,current_node[0]):          
               path_coordinates = current_node[1]
               
               return path_coordinates
      else: #local planning 
          if current_node[0] == goal_states[-1]: #taking last element of positions_after_collision array i.e final goal #to be improved
              
              path_coordinates = current_node[1]
              return path_coordinates

      #Expand node and get successors
      if planner==1:  #if its planning globally
        successors = problem.getSuccessors(current_node[0])        
      else: #if it wants plan locally
        
        successors= problem.getSuccessors_local(robot_id, current_node[0], collison_point, neighbor_robot_point, start_state)
        
      for successor, action, cost in successors:
          
          g = current_node[2] + cost
          h = heuristic_1(problem, successor, heusritic_weight, robot_id)
          path = current_node[1] + [successor]
          temp_node = [successor, path, g, h+g]
         
          #Check if the successor already exists in explored list
          if successor in explored:
              continue       #If so already optimal do not add to list
          
          #Check if duplicate node exists in fringe
          flag_do_not_append = False
          for node in fringe:        
              if node[0] == successor:                  
                   #Check if existing duplicate is actually shorter path than the new node            
                  if node[2] <= temp_node[2]:
                      #In this case do not add the new node to fringe 
                      flag_do_not_append = True
                      #No need to check further in existing fringe
                      break
                      
          if flag_do_not_append:
              #In this case do not add the new node 
              continue
          
          #If none of the above then add successor to fringe 
          
          fringe.append(temp_node)
          
         
  return ([])  

