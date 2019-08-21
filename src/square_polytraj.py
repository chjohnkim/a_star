#!/usr/bin/env python2
from __future__ import print_function

import argparse
import rospy
import sys
import math
import numpy as np

square = np.array([[0.5, 0.5, 1], [0.5, -0.5, 1], [1.5, -0.5, 1], [1.5, 0.5, 1]])
number_of_cycles = 3 # Number of cycles
if number_of_cycles == 1:
    waypoints = square
else:
    for i in range(number_of_cycles-1):
        if i == 0:
            waypoints = np.concatenate((square, square), axis=0)
        else:
            waypoints = np.concatenate((waypoints, square), axis=0)

n_segments = number_of_cycles * 4 # Number of segments
d = 1 # Distance of each segment
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
            B[i][1][j] = B[i][0][j]
            B[i][3][j] = 0
        B[i][2][j] = 0
        B[i][4][j] = 0 
        B[i][5][j] = 0
        if i > 0:
            B[i][2][j] = B[i-1][3][j]
        np.transpose(C[i])[j] = np.transpose(np.linalg.solve(A, np.transpose(B[i])[j]))

print(C[0])
print(np.transpose(C[0])[0])
print(np.transpose(C[0])[1])
print(np.transpose(C[0])[2])
#test = np.dot(A,np.transpose(C[1])[1])
#print(test)
#print(np.transpose(B[1])[1])
#print(np.transpose(np.transpose(C[0])[0]))
#print(np.transpose(C[0])[0])


t = 0

while t<100:
    segment_number = int(t/time_per_segment)
    segment_section = segment_number%4
    time_in_segment = t%time_per_segment
    #print(segment_number, segment_section, time_in_segment)
    

    x = np.dot([time_in_segment**5, time_in_segment**4, time_in_segment**3, time_in_segment**2, time_in_segment, 1], np.transpose(C[segment_section+4])[0])
    y = np.dot([time_in_segment**5, time_in_segment**4, time_in_segment**3, time_in_segment**2, time_in_segment, 1], np.transpose(C[segment_section+4])[1])
    vx = np.dot([5*time_in_segment**4, 4*time_in_segment**3, 3*time_in_segment**2, 2*time_in_segment, 1, 0], np.transpose(C[segment_section+4])[0])
    vy = np.dot([5*time_in_segment**4, 4*time_in_segment**3, 3*time_in_segment**2, 2*time_in_segment, 1, 0], np.transpose(C[segment_section+4])[1])
    ax = 0
    ay = 0 
    t = t + 0.02
    print(x,y, vx, vy)
    
