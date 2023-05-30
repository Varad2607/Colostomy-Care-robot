#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String

"""
Navigation Node
    + Based on the user's signal, start navigation of robot to the desired location

    + navigate_to_patient
        - send /initialpose to navigation for localization
        - send /move_base/goal as the goal location & orientation
    
    + navigate_to_bin
        - send move_base/goal as the goal location & orientation
        - send signal to saved-poses to perform throwing away
    
    + navigate_to_initial
        - return to the initial position
"""

class NavigationNode:
    # Define initial position and orientation
    initial_pos_x = -0.07314966748874041
    initial_pos_y = -0.4065461975616638
    initial_ori_z = 0.12173599926099654
    initial_ori_w =  0.9925625151515277

    # Needed for Stop button updating goal to be current position
    pos_x = 0
    pos_y = 0
    ori_z = 0
    ori_w = 0

    # Define goal position and orientation
    goal_pos_x = 0
    goal_pos_y = 0
    goal_ori_z = 0
    goal_ori_w = 0

    def __init__(self):
        rospy.init_node('navigation_node')

        # Create publishers for sending goals
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.movebase_goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

        # Subscribe to topics for receiving signals
        rospy.Subscriber('/colostomy_care/navigation', String, self.navigation_callback)
        rospy.Subscriber('amcl/pose', PoseWithCovarianceStamped, self.pose_callback)

        # Set the rate at which to publish messages (adjust as needed)
        self.rate = rospy.Rate(1)

    def navigate_to_patient(self):
        # Send /initialpose to navigation for localization
        initial_pose = PoseWithCovarianceStamped()

        # Set the position and orientation values
        initial_pose.pose.pose.position.x = self.initial_pos_x
        initial_pose.pose.pose.position.y = self.initial_pos_y
        initial_pose.pose.pose.orientation.z = self.initial_ori_z
        initial_pose.pose.pose.orientation.w = self.initial_ori_w
        print("Publishing initial pose")
        self.initialpose_pub.publish(initial_pose)
        rospy.sleep(1)  # Wait for the message to be published

        # Set the goal position and orientation for navigation to the patient
        print("Navigating to patient")
        self.goal_pos_x = 1.029626459506202
        self.goal_pos_y = 2.4393594588558365
        self.goal_ori_z = 0.6611025720792328
        self.goal_ori_w = 0.7502955345663619
        self.publishMoveBaseGoal(self.goal_pos_x, self.goal_pos_y, self.goal_ori_z, self.goal_ori_w)

    def navigate_to_bin(self):
        # Set the goal position and orientation for navigation to the bin
        print("Navigating to bin")
        self.goal_pos_x = 1.689220564283644
        self.goal_pos_y = -0.38297850459568644
        self.goal_ori_z = 0.015562107355612882
        self.goal_ori_w = 0.9998789030750935
        self.publishMoveBaseGoal(self.goal_pos_x, self.goal_pos_y, self.goal_ori_z, self.goal_ori_w)

    def navigate_to_initial(self):
        # Set the goal position and orientation for returning to the initial position
        print("Navigating back to initial position")
        self.goal_pos_x = self.initial_pos_x
        self.goal_pos_y = self.initial_pos_y
        self.goal_ori_z = self.initial_ori_z
        self.goal_ori_w = self.initial_ori_w
        self.publishMoveBaseGoal(self.goal_pos_x, self.goal_pos_y, self.goal_ori_z, self.goal_ori_w)

    def publishMoveBaseGoal(self, pos_x, pos_y, ori_z, ori_w):
        # Publish the move_base goal with the given position and orientation
        goal_pose = PoseStamped()
        # Set the goal pose details
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = pos_x
        goal_pose.pose.position.y = pos_y
        goal_pose.pose.orientation.z = ori_z
        goal_pose.pose.orientation.w = ori_w

        movebase_goal = MoveBaseActionGoal()
        movebase_goal.header.stamp = rospy.Time.now()
        movebase_goal.header.frame_id = ''
        movebase_goal.goal.target_pose = goal_pose

        self.movebase_goal_pub.publish(movebase_goal)

    def stop_navigation(self):
        # Stop the robot by updating the goal to the current position
        print("Stopping robot")
        self.publishMoveBaseGoal(self.pos_x, self.pos_y, self.ori_z, self.ori_w)

    def resume_navigation(self):
        # Stop the robot by updating the goal to the current position
        print("Resuming robot")
        self.publishMoveBaseGoal(self.goal_pos_x, self.goal_pos_y, self.goal_ori_z, self.goal_ori_w)

    def pose_callback(self, msg):
        # Callback function for updating the current position and orientation of the robot
        pose = msg.pose.pose
        self.pos_x = pose.position.x
        self.pos_y = pose.position.y
        self.ori_z = pose.orientation.z
        self.ori_w = pose.orientation.w

    def navigation_callback(self, msg):
        nav_goal = msg.data
        print("Navigation Goal: ", nav_goal)
        if nav_goal == "patient":
            self.navigate_to_patient()
        elif nav_goal == "bin":
            self.navigate_to_bin()
        elif nav_goal == "initial":
            self.navigate_to_initial()
        elif nav_goal == "stop":
            self.stop_navigation()
        elif nav_goal == "resume":
            self.resume_navigation()

    def run(self):
        # Run the navigation node
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        nav_node = NavigationNode()
        nav_node.run()
    except rospy.ROSInterruptException:
        pass
