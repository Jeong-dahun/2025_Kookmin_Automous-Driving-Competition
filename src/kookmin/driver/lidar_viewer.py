#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_aspect('equal')
lidar_points, = ax.plot([], [], 'bo')
ranges = None

def callback(data):
    global ranges
    ranges = data.ranges[0:360]

def main():
    global ranges
    rospy.init_node('lidar_visualizer', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    plt.ion()
    plt.show()
    rate = rospy.Rate(10) 
    print("Lidar Visualizer Ready ----------")
    
    while not rospy.is_shutdown():
        if ranges is not None:            
            angles = np.linspace(0,2*np.pi, len(ranges))+np.pi/2
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            lidar_points.set_data(x, y)
            fig.canvas.draw_idle()
            plt.pause(0.01)         
        rate.sleep()

if __name__ == '__main__':
    main()

