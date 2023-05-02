import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import stretch_body.base

# Initialize ROS node
rospy.init_node('aruco_follower')

# Set up the cv_bridge object
bridge = CvBridge()


b = stretch_body.base.Base()
b.left_wheel.disable_sync_mode()
b.right_wheel.disable_sync_mode()
if not b.startup():
    exit() # failed to start base!

# Subscribe to the camera topic
def image_callback(msg):
    # Convert the ROS image message to OpenCV format
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Set up the camera for Aruco marker detection
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect the marker in the frame
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

    # If the marker is detected
    if len(corners) > 0:
        # Calculate the marker center
        center = np.mean(corners[0], axis=0)
        
        # Check if the marker is aligned with the center of the frame
        if center[0][0] < 500 and center[0][1] < 400:
            # Move the robot to the right
            b.rotate_by(-1.57)
            b.push_command()
            b.left_wheel.wait_until_at_setpoint()
            
        elif center[0][0] > 500 and center[0][1] < 400:
            # Move the robot to the left
            b.rotate_by(1.57)
            b.push_command()
            b.left_wheel.wait_until_at_setpoint()
                
        else:
            # Move the robot forward
            b.translate_by(0)
            b.push_command()
            b.left_wheel.wait_until_at_setpoint()   
            
            # Check if the top-left and top-right corners of the marker are at the top of the frame
            if (corners[0][0][0][1] <= 0) and (corners[0][0][1][1] <= 0):
                b.translate_by(0)
                b.push_command()
                b.left_wheel.wait_until_at_setpoint()
        # Draw the detected markers on the frame
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    else:
        b.translate_by(0)
        b.push_command()
        b.left_wheel.wait_until_at_setpoint()

    # Display the camera feed
    cv2.imshow("Camera Feed", frame)
    cv2.waitKey(1)

# Subscribe to the camera topic and continuously process image frames
rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
rospy.spin()

# Close all windows
cv2.destroyAllWindows()


