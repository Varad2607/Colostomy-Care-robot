#!/usr/bin/env python3

import rospy
import math
import tf
import actionlib
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, String

class TeleopNode:

    def __init__(self):
        rospy.init_node('teleop_node')

        # Create publishers for sending goals
        self.movebase_goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

        # Subscribe to topics for receiving signals
        # x,y, ori_z, ori_w(fixed to 1)
        rospy.Subscriber('/teleop_forward', Empty, self.teleop_forward_callback)
        #rospy.Subscriber('/teleop_backward', Empty, self.teleop_backward_callback)
        rospy.Subscriber('/teleop_rot_left', Empty, self.teleop_rot_left_callback)
        rospy.Subscriber('/teleop_rot_right', Empty, self.teleop_rot_right_callback)


        # Initialize the move_base action client
        self.moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.moveBaseClient.wait_for_server()

        # Set the rate at which to publish messages (adjust as needed)
        self.rate = rospy.Rate(10)

    def publishMoveBaseGoal(self, pos_x, pos_y, ori_z, ori_w):
        # Publish the move_base goal with the given position and orientation
        goal_pose = PoseStamped()
        # Set the goal pose details
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.pose.position.x = pos_x
        goal_pose.pose.position.y = pos_y
        goal_pose.pose.orientation.z = ori_z
        goal_pose.pose.orientation.w = ori_w

        movebase_goal = MoveBaseActionGoal()
        movebase_goal.header.stamp = rospy.Time.now()
        movebase_goal.header.frame_id = ''
        movebase_goal.goal.target_pose = goal_pose

        self.movebase_goal_pub.publish(movebase_goal)


    def teleop_forward_callback(self, msg):
        self.publishMoveBaseGoal(0.2, 0, 0, 1)

    def teleop_backward_callback(self, msg):
        self.publishMoveBaseGoal(-0.2, 0, 0, 1)

    def teleop_rot_left_callback(self, msg):
        self.publishMoveBaseGoal(0, 0, 0.2, 1)

    def teleop_rot_right_callback(self, msg):
        self.publishMoveBaseGoal(0, 0, -0.2, 1)

    
    def run(self):
        # Run the navigation node
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        teleop_node = TeleopNode()
        teleop_node.run()
    except rospy.ROSInterruptException:
        pass
