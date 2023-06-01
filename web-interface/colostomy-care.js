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

//   #['joint_lift', 'wrist_extension', 'joint_gripper_finger_left', 'joint_wrist_yaw']
//   # joints and limits
//   # "wrist_extension": [0, .518],
//   # "joint_wrist_yaw": [-1.38, 4.58],
//   # "joint_lift": [0.15, 1.1],
//   # "translate_mobile_base": [-30.0, 30.0],
//   # "rotate_mobile_base": [-3.14, 3.14],
//   # "joint_gripper_finger_left": [-0.375, 0.166] 0.166= open

const teleopJointIncTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/teleop_joint_inc',
    messageType: 'std_msgs/String'
});

const teleopJointDecTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/teleop_joint_dec',
    messageType: 'std_msgs/String'
});

function sendPoseGoal(jointName, jointValue){
    const actionGoal = new ROSLIB.Message({
        goal: {
          trajectory: {
            joint_names: [jointName],
            points: [
              {
                positions: [jointValue],
                time_from_start: { sec: 0, nsec: 0 },
              },
            ],
          },
        },
        goal_id: {
          stamp: { sec: 0, nsec: 0 },
          id: '',
        },
      });
  
      teleOpPublisher.publish(actionGoal);
}

// Teleoperating by sending cmd_vel
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