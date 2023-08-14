#!/usr/bin/env python
# 13/08/2023 Christopher Yip
# Simple obstacle avoidance for Turtlebot3

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

angular_speed = 0.5  # Angular speed for turning
min_distance = 0.3  # Minimum distance to consider an obstacle
current_distance = 0.0 # Default value
count = 0

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.twist = Twist()
        
    def scan_callback(self, msg):
        
	# Get scan data for the front region (approximately 180 degrees)
        current_distance = min(msg.ranges[45:226])
        self.avoid_obstacle()
        rospy.loginfo(current_distance - min_distance)
        
    def avoid_obstacle(self):
        if (current_distance < min_distance):
	    count = count + 1
	    if count > 3:
            	self.twist.linear.x = 0.0
            	self.twist.angular.z = angular_speed
            	self.twist_pub.publish(self.twist)
	    	rospy.loginfo("avoiding")
		count = 0
        else:
            self.twist.linear.x = 0.2  # Move forward
            self.twist.angular.z = 0.0
            self.twist_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
