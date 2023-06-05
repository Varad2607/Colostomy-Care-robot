//Create a new ROS instance and connect to the server
var ros = new ROSLIB.Ros({
    //url : 'ws://172.28.7.121:9090'
    url : 'ws://localhost:9090'
});

// Event handler for successful connection
ros.on('connection', function() {
    document.getElementById("status").innerHTML = "Connected";
});

// Event handler for connection errors
ros.on('error', function(error) {
    document.getElementById("status").innerHTML = "Error";
});

// Event handler for connection closure
ros.on('close', function() {
    document.getElementById("status").innerHTML = "Closed";
});

document.addEventListener("DOMContentLoaded", function() {
    document.getElementById("openPopup1").addEventListener("click", function() {
        document.getElementById("popup").style.display = "block";
    })
})
document.addEventListener("DOMContentLoaded", function() {
    document.getElementById("openPopup2").addEventListener("click", function() {
        document.getElementById("popup").style.display = "block";
    })
})

document.addEventListener("DOMContentLoaded", function() {
    document.getElementById("closePopup").addEventListener("click", function() {
        document.getElementById("popup").style.display = "none";
    })
})

////////////////////////////////////////////////////////////////////////////
// Fixed Navigations
////////////////////////////////////////////////////////////////////////////
const navigationTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/colostomy_care/navigation',
    messageType: 'std_msgs/String'
});

function navigateTo(nav_goal) {
    console.log("Navigation goal: " + nav_goal);
    const message = new ROSLIB.Message({
        data: nav_goal
    });
    navigationTopic.publish(message);
}

////////////////////////////////////////////////////////////////////////////
// Aruco Navigations
////////////////////////////////////////////////////////////////////////////
const arucoNavigationTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/colostomy_care/aruco_navigation',
    messageType: 'std_msgs/String'
});

function arucoNavigation(aruco_nav_goal) {
    console.log("Aruco Navigation goal: " + aruco_nav_goal);
    const message = new ROSLIB.Message({
        data: aruco_nav_goal
    });
    arucoNavigationTopic.publish(message);
}

////////////////////////////////////////////////////////////////////////////
// Saved Pose
////////////////////////////////////////////////////////////////////////////
const savedPoseTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/colostomy_care/saved_pose',
    messageType: 'std_msgs/String'
});

function savedPose(pose_name) {
    console.log("Calling saved pose: " + pose_name);
    const message = new ROSLIB.Message({
        data: pose_name
    });
    savedPoseTopic.publish(message);
}

////////////////////////////////////////////////////////////////////////////
// Teleoperating joints
//
// 
// Joints and their limits:
// ------------------------
//   "wrist_extension"            
//   "joint_wrist_yaw"             
//   "joint_lift"                
//   "translate_mobile_base"      
//   "rotate_mobile_base"        
//   "joint_gripper_finger_left" 
//   "joint_head_pan" 
//   "joint_head_tilt"
////////////////////////////////////////////////////////////////////////////

const teleopJointIncTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/colostomy_care/teleop_joint_inc',
    messageType: 'std_msgs/String'
});

const teleopJointDecTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/colostomy_care/teleop_joint_dec',
    messageType: 'std_msgs/String'
});

const stopJointsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/colostomy_care/teleop_joint_stop',
    messageType: 'std_msgs/Empty'
});

function stopJoint() {
    console.log("Stopping joints")
    message = new ROSLIB.Message({});
    stopJointsTopic.publish(message);
}

function incJoint(joint_name) {
    console.log("Incrementing: " + joint_name)
    message = new ROSLIB.Message({
        data: joint_name
    });
    teleopJointIncTopic.publish(message);
}

function decJoint(joint_name) {
    console.log("Decrementing: " + joint_name)
    message = new ROSLIB.Message({
        data: joint_name
    });
    teleopJointDecTopic.publish(message);
}
////////////////////////////////////////////////////////////////////////////
// Teleoperating by sending cmd_vel
////////////////////////////////////////////////////////////////////////////
var cmdVelTopic= new ROSLIB.Topic({
    ros : ros,
    name : '/stretch/cmd_vel',
    messagType : 'geometry_msgs/Twist'
  });

function move (linear_x, angular) {
    console.log("in move")
    var twist = new ROSLIB.Message({
      linear: {
        x: linear_x,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: angular
      }
    });
    cmdVelTopic.publish(twist);
  }