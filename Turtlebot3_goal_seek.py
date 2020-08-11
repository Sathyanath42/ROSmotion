#!/usr/bin/env python  
import rospy
from math import radians, degrees
from geometry_msgs.msg import Point
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *

#this method will make the robot move to the goal location
def go_to_goal(xGoal,yGoal):

   #define a client for to send goal requests to the move_base server through a SimpleActionClient
   ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

   #wait for the action server
   while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
           rospy.loginfo("Waiting for the action server")

   #Assign an object of the type MoveBaseGoal(0)
   endpoint = MoveBaseGoal()
      
   #set up the reference frame parameters
   endpoint.target_pose.header.frame_id = "map"
   endpoint.target_pose.header.stamp = rospy.Time.now()

   #Define the position values and the orientation values
   endpoint.target_pose.pose.position =  Point(xGoal,yGoal,0)
   endpoint.target_pose.pose.orientation.x = 0.0
   endpoint.target_pose.pose.orientation.y = 0.0
   endpoint.target_pose.pose.orientation.z = 0.0
   endpoint.target_pose.pose.orientation.w = 1.0

   rospy.loginfo("Feeding the goal location")
   ac.send_goal(endpoint)

   #Assgin a execution time of 90 seconds
   ac.wait_for_result(rospy.Duration(90))

   #Check status of the goal seek operation
   if(ac.get_state() ==  GoalStatus.SUCCEEDED):
           rospy.loginfo("Destination reached")
           return True
   else:
           rospy.loginfo("Failed to reach the destination")
           return False

if __name__ == '__main__':
   rospy.init_node('tb3_navigation', anonymous=False)
   x_goal = -5.97611474991
   y_goal = 4.00300264359
   print'moving to goal'
   go_to_goal(x_goal,y_goal)
   rospy.spin()
