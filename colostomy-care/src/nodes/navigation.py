#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, String

"""
Navigation Node
    + Based on the user's singnal, start navigation of robot to the desired location

    + navigate_to_patient
        - send /initialpose to navigation for localization
        - send /move_base/goal as the goal location & orientation
    
    + navigate_to_bin
        - send move_base/goalas the goal location & orientation
        - send singal to saved-poses to perform thowing away
    
    + navigate_to_initial
        - return to the inital position
"""

class NavigationNode:
    initial_pos_x = 0.8601552248001099
    initial_pos_y = -3.261725425720215
    initial_ori_z = 0.5800450634824666
    initial_ori_w = 0.8145843874821204

    pos_x = 0
    pos_y = 0
    ori_z = 0
    ori_w = 0

    def __init__(self):
        rospy.init_node('navigation_node')

        # Create publishers for sending goals
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.movebase_goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.throwing_signal_pub = rospy.Publisher('/throwing_signal', String, queue_size=10)

        rospy.Subscriber('/start_navigation_to_patient', Empty, self.start_navigation_patient_callback)
        rospy.Subscriber('/start_navigation_to_bin', Empty, self.start_navigation_bin_callback)
        rospy.Subscriber('/start_navigation_to_initial', Empty, self.start_navigation_initial_callback)
        rospy.Subscriber('/stop_navigation', Empty, self.stop_navigation_callback)
        rospy.Subscriber('amcl/pose', PoseWithCovarianceStamped, self.pose_callback)

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
        print("Publishing initial post")
        self.initialpose_pub.publish(initial_pose)
        rospy.sleep(1)  # Wait for the message to be published
        self.publishMoveBaseGoal(0.5521001815795898, 4.632573127746582, 0.9748691112372452, 0.22277840100760127)

    def navigate_to_bin(self):
        self.publishMoveBaseGoal(2.9260339736938477, 4.150063991546631, -0.14516202694640776, 0.9894078966396066)

    def navigate_to_initial(self):
        self.publishMoveBaseGoal(self.initial_pos_x, self.initial_pos_y, self.initial_ori_z, self.initial_ori_w)

    def publishMoveBaseGoal(self, pos_x, pos_y, ori_z, ori_w):
        # Return to the initial position
        goal_pose= PoseStamped()
        # Set the initial pose details

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
        print("Stopping robot")
        self.publishMoveBaseGoal(self.pos_x, self.pos_y, self.ori_z, self.ori_w)

    def stop_navigation_callback(self, msg):
        self.stop_navigation()

    def start_navigation_patient_callback(self, msg):
        self.navigate_to_patient()

    def start_navigation_bin_callback(self, msg):
        self.navigate_to_bin()

    def start_navigation_initial_callback(self, msg):
        self.navigate_to_initial()

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.pos_x = pose.position.x
        self.pos_y = pose.position.y
        self.ori_z = pose.orientation.z
        self.ori_w = pose.orientation.w

    def run(self):
        while not rospy.is_shutdown():
            # Based on the user's signal, start navigation of the robot to the desired location
            user_signal = input("Enter signal (patient/bin/initial/stop): ")

            if user_signal == "patient":
                self.navigate_to_patient()
            elif user_signal == "bin":
                self.navigate_to_bin()
            elif user_signal == "initial":
                self.navigate_to_initial()
            elif user_signal == "stop":
                self.stop_nvaigation_callback()
            else:
                rospy.logwarn("Invalid signal entered.")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        nav_node = NavigationNode()
        nav_node.run()
    except rospy.ROSInterruptException:
        pass