#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy, time
from sensor_msgs.msg import LaserScan

ranges = None

def lidar_callback(data):
    global ranges    
    ranges = data.ranges[0:360]

rospy.init_node('Lidar', anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

rospy.wait_for_message("/scan", LaserScan)
print("Lidar Ready ----------")

while not rospy.is_shutdown():
        
    step = (len(ranges) // 36) * 2
    print("Distance:", [round(d,1) for d in ranges[::step]])
    time.sleep(0.5)
    
