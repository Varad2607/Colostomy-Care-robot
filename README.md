# [Colostomy Care](https://sites.google.com/view/colostomy-care-robot/home)
This is the code repo for out Colostomy Care Capstone project for CSE 481C (2023 Spring), operating on the Stretch-2 robot.

## Components
`web-interface/colostomy-care-web-interface.html`: is the main UI for the user to control, it connects to the Stretch Robot IP, this website can be ran on local computer after setting up the ROS_MASTER_URI

`colostomy-care` is the ROS package that we created it contains the following:
  - `saved-poses.py`: performed saved poses to align with colostomy bag, remove it, throw it, and stow joints.
  - `navigation.py`: given an initial position, navigate to the patient, and the bin. These positions are currently ard coded, you can adjust the hard coded values after reading the values from /amcl_pose topic (after giving initial pose and navigation goal to the desired position after running the hello robot's navigation.launch).
  - `teleop.py`: allow the controls of the joins and the base of the robot.

For recording new poses: we utilize the `testing/saved-poses-by-demo.py`.

## To run the Colostomy-Care process
1. Start the stretch_ros's navigation stack on the robot, colostomy-care.yaml is created using [hello robot's navigation tutorial](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/navigation_stack/)\
  `roslaunch stretch_navigation navigation.launch map_yaml:=${HELLO_FLEET_PATH}/maps/colostomy-care.yaml`
3. Start the rosbridge_websocket, to allow the web interface to connect with the robot\
`roslaunch rosbridge_server rosbridge_websocket.launch`
5. Launch file to start our nodes on the robot\
 `roslaunch colostomy-care colostomy-care.launch`
