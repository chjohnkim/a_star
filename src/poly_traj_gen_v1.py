#!/usr/bin/env python2
from __future__ import print_function

import argparse
import rospy
import sys
import math
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50) 
marker_pub = rospy.Publisher("obstacles", Marker, queue_size=10)
rospy.init_node('trajectory', anonymous=True)

def mark_reference(marker_pub, position, color, id):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = id
    marker.type = 3
    marker.action = marker.ADD
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    if color is "red":
        marker.color.a = 1.0
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
    if color is "green":
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
    if color is "blue":
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
    marker.lifetime = rospy.Duration()
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker_pub.publish(marker)

#A* shortest path algorithm
#Input: Map - An array of 3 element vectors where first vector is start coordinate, last vector is end coordinate, and intermediate vectors are obstacle coordinates. 
#Output: Waypoints of shortest path as an array of 3 element vectors
def path_from_A_star(map):

    #Boundaries of map    
    MIN_X = -10
    MIN_Y = -10
    MAX_X = 10
    MAX_Y = 10

    source = np.array(map[0])
    destination = np.array(map[-1])
    obstacles = np.array(map[1:-1])
    visited = [[0,0,0,0,0,0,0]]
    
    #print("source")
    #print(source)
    #print("obstacles")
    #print(obstacles)
    #print("destination")
    #print(destination)

    # Set of all nodes in Map Space
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

        # 6. If the destination node has been marked visited, then terminate.
        if np.array_equal(visited[-1][0:3], destination):
            break

        # 7. Otherwise, select the unvisited node with the smallest tentative
        # distance, set it as the new current node, and go back to Step #4.
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



map1 = ([[1.0, 1., 1.], # start point
         [1.0, 2., 1.], # obstacle
         [3.0, 3., 1.], #    .
         [4.0, 3., 1.], #    .
         [1, 5., 1.], #    .
         [3, 5., 1.], #    .
         [2, 7., 1.], # obstacle
         [2, 9., 1.]]) # target point

map2 = ([[4, 9, 1], # start point
         [1, 1, 1]]) # target point

map3 = ([[-5, -7,  1], # start point
         [-4,  1,  1], # obstacle
         [ 1,  2,  1], # obstacle
         [ 3,  3,  1], #    
         [ 3,  5,  1], #    .
         [ 2,  7,  1], # obstacle
         [-3, -2,  1], #    .
         [ 1,  1,  1], #    .
         [ 1,  5,  1], #    .
         [ 3,  7,  1], #    .
         [ 0,  0,  1], # obstacle
         [ 2,  1,  1], # obstacle
         [ 3,  3,  1], #    .
         [ 1,  3,  1], #    .
         [ 2,  5,  1], #    .
         [ 4,  5,  1], #    .
         [-1, -2,  1], # obstacle
         [-3, -3,  1], #    .
         [-4,  3,  1], #    .
         [-3, -5,  1], #    .
         [ 2, -7,  1], # obstacle
         [-3, -2,  1], #    .
         [ 1, -1,  1], #    .
         [-1, -5,  1], #    .
         [-3, -7,  1], #    .
         [ 0,  0,  1], # obstacle
         [-2, -1,  1], # obstacle
         [-3, -3,  1], #    .
         [-1, -3,  1], #    .
         [-2,  5,  1], #    .
         [ 3,  7,  1], # obstacle
         [ 2,  4,  1]]) # target point

test_map = map2

waypoints = path_from_A_star(test_map)

print("A* path: ")
print(waypoints)


n_segments = len(waypoints)-1 # Number of segments
#d = 1 # Distance of each segment for calculating time per segment, but omit since the distance is either 1 or 1.414 anyways
time_per_segment = 2.5 # Time per segment is 2.5 s
time_interval = np.zeros((n_segments+1, 1))
for i in range(len(time_interval)):
    time_interval[i] = i*time_per_segment

T = time_per_segment
A = np.array([[0,       0,       0,      0,    0, 1], 
              [T**5,    T**4,    T**3,   T**2, T, 1],
              [0,       0,       0,      0,    1, 0], 
              [5*T**4,  4*T**3,  3*T**2, 2*T,  1, 0], 
              [0,       0,       0,      2,    0, 0],
              [20*T**3, 12*T**2, 6*T,    2,    0, 0]])

B = np.zeros((n_segments,6,3))
C = np.zeros((n_segments,6,3))

for i in range(n_segments):
    for j in range(3):
        B[i][0][j] = waypoints[i][j]
        if i < n_segments-1:
            B[i][1][j] = waypoints[i+1][j]
            B[i][3][j] = ((waypoints[i+1][j]-waypoints[i][j])/time_per_segment)/math.sqrt(2)
        else:
            B[i][1][j] = waypoints[i+1][j]
            B[i][3][j] = 0
        B[i][2][j] = 0
        B[i][4][j] = 0 
        B[i][5][j] = 0
        if i > 0:
            B[i][2][j] = B[i-1][3][j]
        np.transpose(C[i])[j] = np.transpose(np.linalg.solve(A, np.transpose(B[i])[j]))


odom = Odometry()
rate = rospy.Rate(50)  # 50hz
t = 0
while not rospy.is_shutdown():
    segment_number = int(t/time_per_segment)
    time_in_segment = t%time_per_segment
    
    x = np.dot([time_in_segment**5, time_in_segment**4, time_in_segment**3, time_in_segment**2, time_in_segment, 1], np.transpose(C[segment_number])[0])
    y = np.dot([time_in_segment**5, time_in_segment**4, time_in_segment**3, time_in_segment**2, time_in_segment, 1], np.transpose(C[segment_number])[1])
    vx = np.dot([5*time_in_segment**4, 4*time_in_segment**3, 3*time_in_segment**2, 2*time_in_segment, 1, 0], np.transpose(C[segment_number])[0])
    vy = np.dot([5*time_in_segment**4, 4*time_in_segment**3, 3*time_in_segment**2, 2*time_in_segment, 1, 0], np.transpose(C[segment_number])[1])
    ax = 0
    ay = 0

    odom.header.frame_id = "world"
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 1
    odom_pub.publish(odom)

    mark_reference(marker_pub, test_map[0],"blue", id=0)
    for i in range(1,len(test_map)-1):
        mark_reference(marker_pub, test_map[i],"red", id=i)
    mark_reference(marker_pub, test_map[-1],"green", id=100)
    
    
    if t < time_per_segment*n_segments-0.02:
        t = t + 0.02
    else:
        t = time_per_segment*n_segments-0.000001
    rate.sleep()
    