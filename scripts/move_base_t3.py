#!/usr/bin/env python
# Adapted from turtlebot3_nav tutorial

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class MoveBaseT3():

    def __init__(self):

        rospy.init_node('move_base_t3')
  	
        points_seq = [2,0.5,0,1.5,-0.5,0,0.2,0.5,0] 
	yaweulerangles_seq = [90,180,0]
        n = 3
        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0 # waypoint progression counter
	
	# Waypoint sequence with orientation
        for i in range(0,len(yaweulerangles_seq)):
             quart = Quaternion(*(quaternion_from_euler(0, 0, yaweulerangles_seq[i]*math.pi/180, axes='sxyz')))
	     coord = points_seq[i*n : i*n+3]
    	     self.pose_seq.append(Pose(Point(*coord), quart))

	# ActionClient set up
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        self.movebase_client()
    
    # Status call back function 
    def goal_prog(self, status, result):
        self.goal_cnt += 1
	if status == 0:
	    rospy.loginfo("Goal pose "+str(self.goal_cnt)+" pending...")

        if status == 3:
            rospy.loginfo("Waypoint "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.goal_prog) 
            else:
                rospy.loginfo("Final waypoint reached!")
		rospy.loginfo("Restarting sequence ...")
                self.goal_cnt = 0
                self.movebase_client()

        if status == 4 or status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted/rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

	if status == 6 or status == 7:
            rospy.loginfo("Cancel requested ")
            rospy.signal_shutdown("Successfully cancelled, shutting down!")
            return

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        self.client.send_goal(goal, self.goal_prog)

if __name__ == '__main__':
    try:
        mbt3 = MoveBaseT3()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminated.")


