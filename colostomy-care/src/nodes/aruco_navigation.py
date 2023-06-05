#!/usr/bin/env python3

import rospy
import actionlib
import aruco_navigation_helper as an
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String



class ArucoNavigationNode:


    def __init__(self):
        rospy.init_node('aruco_navigation_node')

        # Subscribe to topics for receiving signals
        rospy.Subscriber('/colostomy_care/aruco_navigation', String, self.aruco_navigation_callback)
    
        aruco = an.ArucoNavigationNode()

        # Set the rate at which to publish messages (adjust as needed)
        self.rate = rospy.Rate(1)

    def aruco_navigation_callback(self, msg):
        aruco_nav_goal = msg.data
        print("Aruco Navigation Goal: ",  aruco_nav_goal)
        if  aruco_nav_goal == "save patient":
            self.aruco.save_pose("patient","colostomy-bag")
        elif aruco_nav_goal == "save bin":
            self.aruco.save_pose("bin","bin")

        elif aruco_nav_goal == "to patient":
            self.aruco.go_to_pose("patient")
        elif aruco_nav_goal == "to bin":
            self.aruco.go_to_pose("bin")

        elif aruco_nav_goal == "clear patient":
            self.aruco.delete_pose("patient")
        elif aruco_nav_goal == "clear bin":
            self.aruco.delete_pose("bin")

    def run(self):
        # Run the navigation node
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        nav_node = ArucoNavigationNode()
        nav_node.run()
    except rospy.ROSInterruptException:
        pass
