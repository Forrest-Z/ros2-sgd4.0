// Connecting to ROS
// -----------------

var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function () {
    console.log('Connected to websocket server.');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
});

var dest_x;
var dest_y;

// Publishing a Topic
// ------------------

//   var cmdVel = new ROSLIB.Topic({
//     ros : ros,
//     name : '/cmd_vel',
//     messageType : 'geometry_msgs/Twist'
//   });

//   var twist = new ROSLIB.Message({
//     linear : {
//       x : 0.1,
//       y : 0.2,
//       z : 0.3
//     },
//     angular : {
//       x : -0.1,
//       y : -0.2,
//       z : -0.3
//     }
//   });
//   cmdVel.publish(twist);

// Subscribing to a Topic
// ----------------------

//   var listener = new ROSLIB.Topic({
//     ros : ros,
//     name : '/listener',
//     messageType : 'std_msgs/String'
//   });

//   listener.subscribe(function(message) {
//     console.log('Received message on ' + listener.name + ': ' + message.data);
//     listener.unsubscribe();
//   });

// Calling a service
// -----------------

function loadMapData() {
    var mapInfoClient = new ROSLIB.Service({
        ros: ros,
        name: '/get_map_info',
        serviceType: 'sgd_msgs/GetMapInfo'
    });

    console.log('Call service');
    var request = new ROSLIB.ServiceRequest({});

    mapInfoClient.callService(request, function (result) {
        const obj = JSON.parse(result.address_json);
        var list = document.getElementById('dropdownList');
        for (let i = 0; i < obj.addresslist.length; i++) {
            var entry = document.createElement('li');
            entry.appendChild(document.createTextNode(obj.addresslist[i].address));
            list.appendChild(entry);
        }
    });
}

function handleGoal() {
    // get destination from input element
    let dest = document.getElementById("dest").value;
    // call service to start path computation
    var computePathClient = new ROSLIB.Service({
        ros: ros,
        name: '/get_global_plan',
        serviceType: 'sgd_msgs/GetGlobalPlan'
    });

    var request = new ROSLIB.ServiceRequest({
        dest_id: dest
    });

    computePathClient.callService(request, function (result) {
        var list = document.getElementById('route_ids');
        for (let i = 0; i < result.waypoints.length; i++) {
            var entry = document.createElement('li');
            entry.appendChild(document.createTextNode(result.waypoints[i].x + ', ' + result.waypoints[i].y));
            dest_x = result.waypoints[i].x;
            dest_y = result.waypoints[i].y;
            list.appendChild(entry);
        }
    });

    document.getElementById('dest_chooser').style = "none";
    document.getElementById('route_preview').style = "display: flex";
}

function startRoute() {
    // send the goal to a topic -> master_control_unit
    // master_control_unit calls action /navigate_to_pose
    // mcu publishes progress over topic

    var cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/goalpose',
        messageType : 'geometry_msgs/Point'
    });

    var point = new ROSLIB.Message({
        x: dest_x,
        y: dest_y,
        z: 0.0
    });

    cmdVel.publish(point);

    document.getElementById('route_preview').style = "none";
    document.getElementById('dir_info').style = "display: flex";
}

function filterFunction() {
    var input, filter, ul, li, a, i;
    input = document.getElementById("dest");
    filter = input.value.toUpperCase();
    ul = document.getElementById("dropdownList");
    a = ul.getElementsByTagName("li");
    for (i = 0; i < a.length; i++) {
        txtValue = a[i].textContent || a[i].innerText;
        if (txtValue.toUpperCase().indexOf(filter) > -1) {
            a[i].style.display = "";
        } else {
            a[i].style.display = "none";
        }
    }
}

function dropdownSelect() {
    var x = event.clientX, y = event.clientY,
        elementMouseIsOver = document.elementFromPoint(x, y);
    var inp = document.getElementById("dest");
    inp.value = elementMouseIsOver.textContent;
}

function delay(time) {
    return new Promise(resolve => setTimeout(resolve, time));
}