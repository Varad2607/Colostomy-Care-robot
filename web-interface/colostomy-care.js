//Create a new ROS instance and connect to the server
var ros = new ROSLIB.Ros({
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