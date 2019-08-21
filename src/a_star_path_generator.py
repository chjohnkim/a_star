#!/usr/bin/env python2
from __future__ import print_function

import argparse
import rospy
import sys
import math
import numpy as np


map1 = ([[1.0, 1., 1.], # start point
         [1.0, 2., 1.], # obstacle
         [3.0, 3., 1.], #    .
         [4.0, 3., 1.], #    .
         [1, 5., 1.], #    .
         [3, 5., 1.], #    .
         [2, 7., 1.], # obstacle
         [2, 9., 1.]]) # target point


map2 = ([[1, 1, 1], # start point
         [2, 1, 1], # obstacle
         [3, 3, 1], #    .
         [1, 3, 1], #    .
         [2, 5, 1], #    .
         [4, 5, 1], #    .
         [3, 7, 1], # obstacle
         [4, 9, 1]]) # target point

def path_from_A_star(map):
    
    size_map = len(map)
    MIN_X = 0
    MIN_Y = 0
    MAX_X = 10
    MAX_Y = 10

    source = np.array(map[0])
    destination = np.array(map[-1])
    obstacles = np.array(map[1:-1])
    visited = [[0,0,0,0,0,0,0]]
    print("source")
    print(source)
    print("obstacles")
    print(obstacles)
    print("destination")
    print(destination)
    
    # Set of all nodes in Configuration Space
    for i in range(MIN_X, MAX_X+1): #Range 0~MAX_X
        for j in range(MIN_Y, MAX_Y+1): #Range 0~MAX_Y
            
            if i==MIN_X and j==MIN_Y:
                nodes =  np.array([[i, j, 1, float('inf'), 0, 0, 0]])
            else:
                nodes = np.concatenate((nodes, np.array([[i, j, 1, float('inf'), 0, 0, 0]])))    
    
    # Remove nodes with obstacles 
    obstacle_index = []
    for i in range(len(nodes)):
        nodes[i,5] = i+1
        for j in range(len(obstacles)):
            if np.array_equal(nodes[i][0:3], obstacles[j]):
                obstacle_index = np.concatenate((obstacle_index, [i+1]))

    for i in range(len(obstacle_index)):
        real_index = obstacle_index[i]-1-i
        nodes = np.delete(nodes, real_index, 0)
    
    # 2. Assign to every node a tentative distance value. 
    # Set it to 0 for the source and inf to all other nodes. 
    for i in range(len(nodes)):
        if np.array_equal(nodes[i][0:3], source):
            source_index = i  
            break
    nodes[source_index][3] = 0

    # 3. Set the source as current. 
    current_index = source_index
    nodes[current_index][4] = 1
    
    while len(nodes)>0:
        # 4. For the current node, consider all of its unvisited neighbors and
        # calculate their tentative distance through the  current node. Compare
        # the tentative distance to the current distance value and assign the
        # smaller one.
        for i in range(len(nodes)):
            if abs(nodes[i][0]-nodes[current_index][0])<=1 and abs(nodes[i][1]-nodes[current_index][1])<=1 and i!=current_index:
                nodes[i][4] = 2
                temp_distance = nodes[current_index][3] + math.sqrt((nodes[current_index][0]-nodes[i][0])**2 + (nodes[current_index][1]-nodes[i][1])**2) + math.sqrt((nodes[current_index][0]-destination[0])**2 + (nodes[current_index][1]-destination[1])**2)
                if temp_distance < nodes[i][3]:
                    nodes[i][3] = temp_distance
                    nodes[i][6] = nodes[current_index][5] #Record parent node of shortest path
        
        # 5. When we are done considering all the unvisited neighbors of the
        # current node, mark the current node as visited and remove it from the
        # unvisited set. A visited node will never be checked again.
        visited = np.concatenate((visited, [nodes[current_index]]))
        nodes = np.delete(nodes, current_index, 0)

        #nodes(:,5) = 0;
        
        # 6. If the destination node has been marked visited, then terminate.
        if np.array_equal(visited[-1][0:3], destination):
            break

        # 7. Otherwise, select the unvisited node with the smallest tentative
        # distance, set it as the new current node, and go backto Step #4.
        current_index = np.where(nodes.transpose()[3] == np.amin(nodes.transpose()[3]))[0][0] 
        nodes[current_index][4] = 1
    
    visited = np.delete(visited, 0, 0) # Delete initialized value

    # Backtrack the set of nodes with the shortest path and store as Optimal path
    optimal_path = [[0,0,0]]
    parent_index = len(visited)-1
    
    while parent_index != 0:
        optimal_path = np.concatenate(([visited[parent_index][0:3]], optimal_path))    
        parent_index = int(np.where(visited.transpose()[5] == visited.transpose()[6][parent_index])[0][0])
    optimal_path = np.concatenate(([visited[parent_index][0:3]], optimal_path))
    optimal_path = np.delete(optimal_path, -1, 0) # Delete initialized value
       
    return optimal_path
    
print("map1")
print(path_from_A_star(map1))
print("map2")
print(path_from_A_star(map2))
