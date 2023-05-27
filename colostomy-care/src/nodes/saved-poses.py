#!/usr/bin/env python3
"""
Saved pose node
    + Perform the saved poses for the joints

    + Move to bag pose
        - after confirmation of from the user on the navigation
        - call to start the aruco marker adjustment of the gripper

    + grip and remove bag pose
        - after confirmation from the user on the gripper location 
        - perform the removal of the bag and 
        - send signal to front-end to indicate the bag is removed 
        - send singnal to navigation to move to bin
    
    + throw away bag
        - after signal from navigation
        - release gripper
        - send signal to navigation to return to initial position
"""
import stretch_body.arm
import stretch_body.lift
import stretch_body.stretch_gripper as gripper
import time
from stretch_body.hello_utils import *
import stretch_body.wrist_yaw as wrist
import rospy
from std_msgs.msg import Empty
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SavedPosesNode:
    #['joint_lift', 'wrist_extension', 'joint_gripper_finger_left', 'joint_wrist_yaw']
    # joints and limits
    # "wrist_extension": [0, .518],
    # "joint_wrist_yaw": [-1.38, 4.58],
    # "joint_lift": [0.15, 1.1],
    # "translate_mobile_base": [-30.0, 30.0],
    # "rotate_mobile_base": [-3.14, 3.14],
    # "joint_gripper_finger_left": [-0.375, 0.166] 0.166= open
    align_with_bag_poses = {
        'initial lift up ': (0.7, 0, 0.166, 0.08053399136399617),
        'initial': (0.5615861676470746, 0.24211288835998443, 0.166, 0.08053399136399617),
        'at-bag': (0.4795945004727337, 0.24211526939820244, 0.166, 0.08053399136399617)
    }
    remove_bag_poses = {
        'close gripper': (0.479591149126581, 0.2421176766016756, -0.375, 0.08053399136399617),
        'move wrist': (0.479591149126581, 0.2421176766016756, -0.375,  2.14629478571666),
        'lift up': (0.7307844880308441, 0.2421176766016756, -0.375, 2.14629478571666),
        'remove and lift': (0.7307844880308441, 0.0027174925323322544, -0.375, 2.14629478571666)
    }
    throw_bag_poses ={
        'extend to bin': (0.7307844880308441, 0.2614214337317741, -0.375, 2.14629478571666),
        'gripper to face bin': (0.7307844880308441, 0.2614214337317741, -0.375, -0.23648870479903633),
        'release gipper': (0.7307844880308441, 0.2614214337317741, 0.166, -0.23648870479903633),
        'release gipper 2': (0.7307844880308441, 0.2614214337317741, 0.166, -0.23648870479903633),
        'stow': (0.2298394901246486, 0, 0.021475731030398976, 3.397767445166695)
    }

    def __init__(self):
        rospy.init_node('saved_pose_node')

        rospy.Subscriber('/align_with_bag', Empty, self.align_with_bag_callback)
        rospy.Subscriber('/remove_bag', Empty, self.remove_bag_callback)
        rospy.Subscriber('/throw_bag', Empty, self.throw_bag_callback)

        self.jointPublisher = rospy.Publisher('/stretch_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

        # Set the rate at which to publish messages (adjust as needed)
        self.rate = rospy.Rate(1)
        self.w = wrist.WristYaw()
        self.l = stretch_body.lift.Lift()
        self.l.motor.disable_sync_mode()


        self.a = stretch_body.arm.Arm()
        self.a.motor.disable_sync_mode()

        self.g = gripper.StretchGripper()
        print("Starting joints")

        if not (self.l.startup() and self.g.startup(threaded=False) and self.a.startup() and self.w.startup(threaded=False)):
            print("used to exit")
            #exit() # failed to start arm!
        print("Completed Saved Poses Init")

    def performPoses(self, poses):
        for pose_name, pose_pos in poses.items():
            # Get the position values for the pose
            lift_pos, arm_pos, grip_pos, wrist_pos = pose_pos
            # Print the position values for the pose
            print(f"Pose '{pose_name}':")
            print(f"  Lift position: {lift_pos}")
            print(f"  Arm position: {arm_pos}")
            print(f"  Grip position: {grip_pos}")
            print(f"  Wrist position: {wrist_pos}")

            msg = FollowJointTrajectoryActionGoal()

            # Fill in the necessary fields of the message
            msg.goal.trajectory = JointTrajectory()
            msg.goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_gripper_finger_left', 'joint_wrist_yaw']

            point = JointTrajectoryPoint()
            point.positions = [lift_pos, arm_pos, grip_pos, wrist_pos]
            msg.goal.trajectory.points.append(point)

            # Publish the message
            self.jointPublisher.publish(msg)

            # Sleep to allow time for the message to be published
            rospy.sleep(2)
    
    def align_with_bag_callback(self, msg):
        print("Aligning with Bag")
        self.performPoses(self.align_with_bag_poses)

    def remove_bag_callback(self, msg):
        print("Removing Bag")
        self.performPoses(self.remove_bag_poses)

    def throw_bag_callback(self, msg):
        print("Throwing Bag")
        self.performPoses(self.throw_bag_poses)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        saved_pose_node = SavedPosesNode()
        saved_pose_node.run()
    except rospy.ROSInterruptException:
        pass

