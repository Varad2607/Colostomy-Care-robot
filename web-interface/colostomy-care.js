//Create a new ROS instance and connect to the server
var ros = new ROSLIB.Ros({
    url : 'ws://172.28.7.121:9090'
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


const emptyMsg=new ROSLIB.Message({});

const navigateToBinTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/start_navigation_to_bin',
    mesageType: 'std_msgs/Empty'
})

const navigateToPatientTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/start_navigation_to_patient',
    mesageType: 'std_msgs/Empty'
})

const navigateToInitialTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/start_navigation_to_initial',
    mesageType: 'std_msgs/Empty'
})

const stopNavigationTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/stop_navigation',
    mesageType: 'std_msgs/Empty'
})

const resumeNavigationTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/resume_navigation',
    mesageType: 'std_msgs/Empty'
})

const alignWithBagTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/align_with_bag',
    mesageType: 'std_msgs/Empty'
})

const removeBagTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/remove_bag',
    mesageType: 'std_msgs/Empty'
})

const throwBagTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/throw_bag',
    mesageType: 'std_msgs/Empty'
})

const teleopForwardTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/teleop_forward',
    mesageType: 'std_msgs/Empty'
})

const teleopBackwardTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/teleop_backward',
    mesageType: 'std_msgs/Empty'
})

const teleopRotLeftTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/teleop_rot_left',
    mesageType: 'std_msgs/Empty'
})

const teleopRotRightTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/teleop_rot_right',
    mesageType: 'std_msgs/Empty'
})

document.addEventListener("DOMContentLoaded", function() {
    document.getElementById("openPopup").addEventListener("click", function() {
        document.getElementById("popup").style.display = "block";
    })
})

document.addEventListener("DOMContentLoaded", function() {
    document.getElementById("closePopup").addEventListener("click", function() {
        document.getElementById("popup").style.display = "none";
    })
})


const teleOpPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/stretch_controller/follow_joint_trajectory/goal',
    messageType: 'control_msgs/FollowJointTrajectoryActionGoal',
});

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