'''
Robot will nevigate to the center of the room, somewhere away from the bed

1) Perform a scan to find the marker of the bed, try three times

2) saved that value in the maps's frame

3) send the navigation goal of that bag , with some buffer space

4) robot will go to that point


Calculation:

Before transforming to the map's frame

always subtract the y value of the marker pose to give a buffer postion that is left of the patient

'''



import rospy
import tf2_ros
from visualization_msgs.msg import MarkerArray
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

pose_a = None  # colostomy-bag pose
pose_b = None  # wrist_top pose
def aruco_callback(data):
    global pose_a, pose_b
    # Iterate over the markers in the array
    for marker in data.markers:
        if marker.text == "colostomy-bag":
            # Save the pose of colostomy-bag as pose A
            pose_a = marker.pose
            print("found colostomy bag")

        if marker.text == "wrist_top":
            # Save the pose of wrist_top as pose B
            pose_b = marker.pose
            print("wrist top")

    # Check if both pose A and pose B are available
    if pose_a is not None and pose_b is not None:
        # Transform pose B to pose A
        try:
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            
            # Get the transform from pose B to pose A
            transform = tf_buffer.lookup_transform('wrist_top', 'colostomy-bag', rospy.Time(0), rospy.Duration(1.0))
            
            # Apply the transform to pose B
            pose_b_transformed = tf2_geometry_msgs.do_transform_pose(pose_b, transform)

            # Now you have pose A and transformed pose B
            # You can perform further operations or save the transformed pose
            # For example, you can print the transformed pose:
            print("Transformed Pose B: \n%s", pose_b_transformed)
            
        except tf2_ros.TransformException as e:
            print("Transform exception: %s", e)

if __name__ == "__main__":
    rospy.init_node("transform_marker")

    # Create a subscriber to the Aruco marker topic.
    aruco_sub = rospy.Subscriber("/aruco/marker_array", MarkerArray, aruco_callback)

    # Spin until the node is shutdown.
    rospy.spin()
