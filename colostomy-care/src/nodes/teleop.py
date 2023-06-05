#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TeleopNode:
    joint_limits = {
        "wrist_extension": [0.0, 0.518],
        "joint_wrist_yaw": [-1.38, 4.58],
        "joint_lift": [0.15, 1.1],
        "translate_mobile_base": [-30.0, 30.0],
        "rotate_mobile_base": [-3.14, 3.14],
        "joint_gripper_finger_left": [-0.375, 0.166],
        "joint_head_pan": [-3,3],
        "joint_head_tilt": [-1,1]
    }

    def __init__(self):
        rospy.init_node('teleop_node')
        # Create publishers for sending goals
        self.jointPublisher = rospy.Publisher('/stretch_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
        
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/colostomy_care/teleop_joint_stop', Empty, self.teleop_joint_stop_callback)
        rospy.Subscriber('/colostomy_care/teleop_joint_inc', String, self.teleop_joint_inc_callback)
        rospy.Subscriber('/colostomy_care/teleop_joint_dec', String, self.teleop_joint_dec_callback)
        rospy.spin()

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def performPoses(self, joint_name, joint_value):
        msg = FollowJointTrajectoryActionGoal()

        # Fill in the necessary fields of the message
        msg.goal.trajectory = JointTrajectory()
        msg.goal.trajectory.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [joint_value]
        msg.goal.trajectory.points.append(point)

        # Publish the message
        self.jointPublisher.publish(msg)

    def teleop_joint_stop_callback(self, msg):
        joint_state = self.joint_state
        joint_state = self.joint_state

        msg = FollowJointTrajectoryActionGoal()

        msg.goal.trajectory = JointTrajectory()
        msg.goal.trajectory.joint_names = ["wrist_extension", "joint_wrist_yaw","joint_lift","translate_mobile_base"
                                           ,"rotate_mobile_base", "joint_gripper_finger_left", "joint_head_pan",
                                            "joint_head_tilt"]

        point = JointTrajectoryPoint()
        point.positions = [joint_state.position[joint_state.name.index("wrist_extension")],
                           joint_state.position[joint_state.name.index("joint_wrist_yaw")],
                           joint_state.position[joint_state.name.index("joint_lift")],
                           joint_state.position[joint_state.name.index("translate_mobile_base")],
                           joint_state.position[joint_state.name.index("rotate_mobile_base")],
                           joint_state.position[joint_state.name.index("joint_gripper_finger_left")],
                           joint_state.position[joint_state.name.index("joint_head_pan")]]
        
        msg.goal.trajectory.points.append(point)
        # Publish the message
        self.jointPublisher.publish(msg)

    def teleop_joint_inc_callback(self, msg):
        joint_state = self.joint_state
        joint_name = msg.data
        value = joint_state.position[joint_state.name.index(joint_name)]
        
        if joint_name == "joint_lift":
            value += 0.1
        elif joint_name == "joint_gripper_finger_left":
            value = 0.166 # open
        elif joint_name == "joint_wrist_yaw":
            value += 0.3
        elif joint_name == "wrist_extension":
            value += 0.05 
        elif joint_name == "joint_head_tilt":
            value += 0.05 
        elif joint_name == "joint_head_pan":
            value += 0.1 
        else:
            pass
        value = min(value, self.joint_limits[joint_name][1])
        print("Incrementing joint: ", joint_name, " to ", value)
        self.performPoses(joint_name, value)

    def teleop_joint_dec_callback(self, msg):
        joint_state = self.joint_state
        joint_name = msg.data
        value = joint_state.position[joint_state.name.index(joint_name)]
        
        if joint_name == "joint_lift":
            value -= 0.1
        elif joint_name == "joint_gripper_finger_left":
            value = -0.375 # open
        elif joint_name == "joint_wrist_yaw":
            value -= 0.3
        elif joint_name == "wrist_extension":
            value -= 0.05 
        elif joint_name == "joint_head_tilt":
            value -= 0.05 
        elif joint_name == "joint_head_pan":
            value -= 0.1 
        else:
            pass
        value = max(value, self.joint_limits[joint_name][0])
        print("Decrementing joint: ", joint_name, " to ", value)
        self.performPoses(joint_name, value)

if __name__ == '__main__':
    try:
        teleop_node = TeleopNode()
    except rospy.ROSInterruptException:
        pass
