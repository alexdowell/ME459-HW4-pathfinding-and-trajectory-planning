#!/usr/bin/env python3
# -*- coding: utf-8 -*- 


import numpy as np
import math 
import random
from math import atan2
import matplotlib.pyplot as plt # plotting tools





class Node():
    def __init__(self, x,y, cost, index):
        self.x = x
        self.y = y
        self.cost = cost
        self.index = index

class Turtle():
    def __init__(self, x, y, step_size):
        self.position  = [x,y]
        self.move_list = [[step_size,0], #move right
                          [-step_size,0], #move left 
                          [0,step_size], #move up
                          [0,-step_size],#move down
                          [-step_size,-step_size], #move southwest
                          [step_size,-step_size],#move southeast
                          [step_size,step_size],#move northeast
                          [-step_size,step_size]#move northwest
                          ]
        
        self.visited_history = {}
        self.not_visited = {} 
        self.obstacle_location = {}
        
   
            
class ConfigSpace():
    
   # sets up a configuration space based on the following inputs:
   # x_bounds = [x_min,x_max]
   # y_bounds = [y_min,y_max]
   # spacing = grid spacing or step size in between values
    
    def __init__(self, x_bounds, y_bounds, spacing):
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.spacing = spacing
        
    def set_obstacles(self, obstacle_list):
        self.obstacles = obstacle_list
            
    def set_graph_coords(self):
        """graph coordinates and define the search space"""
        self.x_coords = np.arange(self.x_bounds[0], self.x_bounds[1]+self.spacing,
                                  self.spacing)
        
        self.y_coords = np.arange(self.y_bounds[0], self.y_bounds[1]+self.spacing,
                                  self.spacing)
        
        self.generate_search_space()
        
    def get_x_y_coords(self):
        return self.x_coords, self.y_coords
    
    def generate_search_space(self):
        """generate our search space"""
        self.search_space = np.zeros((len(self.x_coords),len(self.y_coords))) 
    
     
    def place_obstacles(self, obst_list):
        """places obstacles in grid by inserting a 1""" 
        for obstacle in obst_list:
            obs_x = obstacle[0]
            obs_y = obstacle[1]
            self.search_space[obs_x, obs_y]= 1
    
    def calc_index(self,position):
        """calculate index """
        index = (position[1] - self.y_bounds[0]) / \
            self.spacing * (self.x_bounds[1] - self.x_bounds[0] + self.spacing)/ \
                self.spacing + (position[0] - self.x_bounds[0]) / self.spacing
                
        return index       
    
#    def calc_index(self, position_x, position_y):
#        """calculate index """
#        index = (position_y - self.y_bounds[0]) / \
#            self.spacing * (self.x_bounds[1] - self.x_bounds[0] + self.spacing)/ \
#                self.spacing + (position_x - self.x_bounds[0]) / self.spacing
#                
#        return index            
    
def check_within_obstacle(obstacle_list, current_position, obstacle_radius):
    """check if I am within collision of obstacle return True if it is
    false if I'm not"""
    for obstacle in obstacle_list:
        distance = compute_distance(current_position, obstacle)
        if distance<=obstacle_radius:
            return True
        else:    
            return False

def check_if_obstacle_is_present(obstacle_list, node_in_question):
    """check to see if an obstacle is in the way"""
    if node_in_question in obstacle_list:
        return True

def check_obstacle_exists(obstacle_list):
    """sanity check to see if obstacle exists"""
    for obst in obstacle_list:
        if configSpace.search_space[obst[0],obst[1]] == 1:
            print("yes", configSpace.search_space[obst[0],obst[1]])

   
def compute_distance(current_pos, another_pos):
    """compute distance"""
    dist = math.sqrt((another_pos[0] - current_pos[0])**2+(another_pos[1]- current_pos[1])**2)
    
    return dist
    #return dist(current_pos, another_pos)

def check_out_bounds( current_position, x_bounds, y_bounds):
        """check out of bounds of configuration space"""
        
        if current_position[0] < x_bounds[0] or current_position[0] > x_bounds[1]:
            return True
        
        if current_position[1] < y_bounds[0] or current_position[1] > y_bounds[1]:
            return True
        
        return False
    
def check_node_validity(obstacle_list, node_in_question, x_bounds, y_bounds, turtle_radius):
    """ check if current node is valid """
    
    if check_out_bounds(node_in_question, x_bounds, y_bounds) == True:
        print("the node in question is out of bounds")
        return False
    
    elif check_if_obstacle_is_present(obstacle_list, node_in_question) == True:
        turtle.obstacle_location[new_index] = new_node
        print("the node in question is an obstacle")
        return False
    
    elif check_within_obstacle(obstacle_list, node_in_question, turtle_radius) == True:
        print("the node in question is too close to an obstacle")
        return False
    
    else:
        print("the node in question is valid")
        return True
def RRT(x_span, y_span,spacing, start_position, goal_point, obstacle_list, obstacle_radius,delta_l):
    
    #%% ##### BUILD WORLD
    configSpace = ConfigSpace(x_span, y_span, spacing)
    configSpace.set_graph_coords()
    
    x_bounds, y_bounds = configSpace.get_x_y_coords()
    configSpace.set_obstacles(obstacle_list)

     ####  RRT  ###
    current_node = Node(start_position[0],start_position[1], 0, -1) # the start node
    node_tree = []
    node_tree.append(current_node)
#    print('tree size : ', len(node_tree))
    index_count = 0
    node_tree_count = 0
    while compute_distance([current_node.x, current_node.y] , goal_point) > delta_l:
        node_tree_count = 1 + node_tree_count
        rand_x = random.randint(x_span[0],x_span[1])
        print('rand_x : ', rand_x)
        rand_y = random.randint(y_span[0],y_span[1])
        print('rand_y : ', rand_y)
        random_point = [rand_x, rand_y]
        node_dist_list = []
        for node in node_tree:
            node_dist = compute_distance([node.x, node.y] , random_point)
            node_and_distance = [node.x, node.y, node.cost, node.index, node_dist]
            node_dist_list.append(node_and_distance)
            if len(node_dist_list)==1:
                min_node = node
                min_node1 = node_and_distance
                #print("been Here")
            if min_node1[4] >= node_and_distance[4]:
                temp_min_node = node
                temp_min_node1 = node_and_distance
                rand_ang = math.atan2((rand_y - temp_min_node.y) , (rand_x - temp_min_node.x))
                print("rand_ang : ", rand_ang)
                print("temp_min_node.x : ", temp_min_node.x)
                print("temp_min_node.y : ", temp_min_node.y)
                temp_new_node_x = (math.cos(rand_ang) * delta_l) + temp_min_node.x
                print(" temp_new_node_x: ",  temp_new_node_x)
                temp_new_node_y = math.sin(rand_ang) * delta_l + temp_min_node.y
                print(" temp_new_node_y: ",  temp_new_node_y)
                
                
                if check_if_obstacle_is_present(obstacle_list, [temp_new_node_x,temp_new_node_y]) == True:
                    print('obstacle',new_index)
                    continue
                
                if check_within_obstacle(obstacle_list, [temp_new_node_x,temp_new_node_y], obstacle_radius) == True:
                    print('node',temp_new_node_x, temp_new_node_y ,'is within obstacle')
                    continue
                
                
                else:
                    
                    #if temp_new_node is valid then:
                    new_node_x = temp_new_node_x
                    new_node_y = temp_new_node_y
                    min_node = node
                    list_position = node_tree.index(node)
                    print('list position is : ', list_position)
                    min_node1 = node_and_distance
        
        new_node_cost = delta_l + min_node.cost
        new_node_parent = list_position
        current_node = Node(temp_new_node_x, temp_new_node_y, new_node_cost, new_node_parent)        
        node_tree.append(current_node)
        print('the node tree has ', len(node_tree), ' nodes in it')
#        if node_tree_count == 5 :  #debug stopper
#            break
    
    dist_from_cur_2_goal = compute_distance([current_node.x, current_node.y] , goal_point)
    cost_2_goal = current_node.cost + dist_from_cur_2_goal
    goal_parent = node_tree.index(current_node)
    print('The goal points parent index is : ', goal_parent)
    goal_node = Node(goal_point[0], goal_point[1], cost_2_goal, goal_parent) 
    node_tree.append(goal_node)
    # While loop
    # return path list
    return node_tree
        #print(node_tree)

        
    ### ### ###
   
#%% Build Astar  


    
    
if __name__=='__main__':
    #set up parameters
    x_span = [0,10]
    y_span = [0,10]
    spacing = .5
    start_position = [0.0,0.0]
    goal_point = [1.0,9.0]

    
    obstacle_list = [[1,1], [4,4], [3,4], [5,0], [5,1], [0,7], [1,7], [2,7], [3,7]] #[[1,1], [4,4], [3,4], [5,0], [5,1], [0,7], [0.5,7], [1,7], [1.5,7], [2,7], [2.5,7], [3,7]]
    obstacle_radius = 0.5
    delta_l = .5
    node_tree = RRT(x_span, y_span,spacing, start_position, goal_point, obstacle_list, obstacle_radius,delta_l)