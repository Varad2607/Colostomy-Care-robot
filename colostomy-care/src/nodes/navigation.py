#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, String

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
    initial_pos_x = 2.5925643420496027
    initial_pos_y = 2.6296681590861994
    initial_ori_z = 0.6291654437834525
    initial_ori_w =  0.7772714097075559

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
        self.throwing_signal_pub = rospy.Publisher('/throwing_signal', String, queue_size=10)

        # Subscribe to topics for receiving signals
        rospy.Subscriber('/start_navigation_to_patient', Empty, self.start_navigation_patient_callback)
        rospy.Subscriber('/start_navigation_to_bin', Empty, self.start_navigation_bin_callback)
        rospy.Subscriber('/start_navigation_to_initial', Empty, self.start_navigation_initial_callback)
        rospy.Subscriber('/stop_navigation', Empty, self.stop_navigation_callback)
        rospy.Subscriber('/resume_navigation', Empty, self.resume_navigation_callback)
        rospy.Subscriber('amcl/pose', PoseWithCovarianceStamped, self.pose_callback)

        # Initialize the move_base action client
        self.moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.moveBaseClient.wait_for_server()

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
        self.goal_pos_x = 0.6796470950400492
        self.goal_pos_y = 5.249431720826594
        self.goal_ori_z = 0.9858776645781445
        self.goal_ori_w = 0.16746710269764503
        self.publishMoveBaseGoal(self.goal_pos_x, self.goal_pos_y, self.goal_ori_z, self.goal_ori_w)

    def navigate_to_bin(self):
        # Set the goal position and orientation for navigation to the bin
        print("Navigating to bin")
        self.goal_pos_x = 3.3016340660936687
        self.goal_pos_y = 3.547072890217289
        self.goal_ori_z = 0.6537889607855791
        self.goal_ori_w = 0.7566769421324482
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

    def stop_navigation_callback(self, msg):
        # Callback function for stopping navigation
        self.stop_navigation()

    def start_navigation_patient_callback(self, msg):
        # Callback function for starting navigation to the patient
        self.navigate_to_patient()

    def start_navigation_bin_callback(self, msg):
        # Callback function for starting navigation to the bin
        self.navigate_to_bin()

    def start_navigation_initial_callback(self, msg):
        # Callback function for starting navigation to the initial position
        self.navigate_to_initial()

    def resume_navigation_callback(self, msg):
        # Resume the robot's navigation by publishing the previous goal
        print("Resume robot")
        self.publishMoveBaseGoal(self.goal_pos_x, self.goal_pos_y, self.goal_ori_z, self.goal_ori_w)

    def pose_callback(self, msg):
        # Callback function for updating the current position and orientation of the robot
        pose = msg.pose.pose
        self.pos_x = pose.position.x
        self.pos_y = pose.position.y
        self.ori_z = pose.orientation.z
        self.ori_w = pose.orientation.w

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
