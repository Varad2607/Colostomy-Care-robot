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


class SavedPosesNode:
    align_with_bag_poses = {
        'initial':(0.5615861676470746, 0.24211288835998443, 4.310486013958652, 0.08053399136399617),
        'at-bag': (0.4795945004727337, 0.24211526939820244, 4.310486013958652, 0.08053399136399617)
    }
    remove_bag_poses = {
        'close gripper': (0.479591149126581, 0.2421176766016756, -7.884661249732196, 0.08053399136399617),
        'remove and lift': (0.7307844880308441, 0.0027174925323322544, -7.884661249732196, 2.14629478571666)
    }
    throw_bag_poses ={
        'throw-bag': (0.7305178374456467, 0.2614214337317741, 4.310486013958652, -0.23648870479903633)
    }

    def __init__(self):
        rospy.init_node('saved_pose_node')

        rospy.Subscriber('/align_with_bag', Empty, self.align_with_bag_callback)
        rospy.Subscriber('/remove_bag', Empty, self.remove_bag_callback)
        rospy.Subscriber('/throw_bag', Empty, self.throw_bag_callback)

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
            self.l.move_to(lift_pos)
            self.l.motor.wait_until_at_setpoint()
            self.l.push_command()
            self.a.move_to(arm_pos)
            self.a.push_command()
            self.a.motor.wait_until_at_setpoint()
            self.w.move_to(wrist_pos)
            time.sleep(2)
            if grip_pos < 0:
                # close
                self.g.move_to(-100)
            else:
                self.g.move_to(100)
            time.sleep(3)
    
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

