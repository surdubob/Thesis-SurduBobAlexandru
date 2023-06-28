const rclnodejs = require('rclnodejs');
const express = require('express');
const path = require('path');
var url = require('url');
const app = express();
const server = require('http').Server(app)
const io = require('socket.io')(server)
const port = 8080;
const motor_speed = 60;

app.use(express.json())

// rclnodejs.init();
// const node = rclnodejs.createNode('MyNode');
// const publisher = node.createPublisher('std_msgs/msg/String', '/web_movement_commands');

var last_map = null;
var last_pose = null;
var last_objects = null;

var ros_node = null;
var goal_publisher = null;

var changeStateClient = null;

rclnodejs.init().then(() => {
  ros_node = new rclnodejs.Node('MyNode');

  ros_node.createSubscription('nav_msgs/msg/OccupancyGrid', '/map', (msg) => {
    // console.log(`Received message: ${typeof msg}`, msg);
    last_map = msg;
  });  

  ros_node.createSubscription('tf2_msgs/msg/TFMessage', '/pose_transforms', (msg) => {
    // console.log(`Received message: ${typeof msg}`, msg);
    msg.transforms.forEach(tr => {
      last_pose = {pose: {pose: {position: tr.transform.translation, orientation: tr.transform.rotation}}}
    });
  });

  ros_node.createSubscription('sensor_msgs/msg/CompressedImage', '/object_detection_image/compressed', (msg) => {
    io.emit('image', msg);
  })

  ros_node.createSubscription('visualization_msgs/msg/MarkerArray', '/detected_object_markers', (msg) => {
    last_objects = msg
  })

  goal_publisher = ros_node.createPublisher('geometry_msgs/msg/PoseStamped', '/goal_pose');

  changeStateClient = ros_node.createClient(
    'path_finder_robot/srv/ChangeState',
    'change_state'
  );

  rclnodejs.spin(ros_node);
});

app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'static/index.html'));
});
app.get('/icon', (req, res) => {
  res.sendFile(path.join(__dirname, 'static/icon.png'));
});

app.get('/get_map', (req, res) => {
  
  res.send(last_map);
});

app.get('/get_pose', (req, res) => {
  res.send(last_pose);
});

app.get('/get_objects', (req, res) => {
  res.send(last_objects);
});

function angleToQuaternion(angleRad) {
  const halfAngle = angleRad * 0.5;
  const sinHalfAngle = Math.sin(halfAngle);
  const cosHalfAngle = Math.cos(halfAngle);

  const x = 0;
  const y = 0;
  const z = sinHalfAngle;
  const w = cosHalfAngle;

  return { x, y, z, w };
}

app.post('/publish_goal_pose', (req, res) => {
  // console.log(req.body)
  const secondsAndNanos = ros_node.getClock().now().secondsAndNanoseconds;
  var stamp = {
    sec: secondsAndNanos.seconds,
    nanosec: secondsAndNanos.nanoseconds,
  };

  var qua = angleToQuaternion(req.body.unghi)
  
  // console.log(qua)

  var pose = {
    header: {
      stamp: stamp,
      frame_id: 'map',
    },
    pose: {
      position: {
        x: req.body.x,
        y: req.body.y,
        z: 0.0
      },
      orientation: qua
    } 
  };

  goal_publisher.publish(pose);
  console.log(pose)
  res.send('OK');
});

app.post('/pick_object', (req, res) => {
  
  changeStateClient.waitForService(100).then((result) => {
    if (!result) {
      console.log('Error: service not available');
      return;
    }
    console.log(req.body)
    const request = {
      state_name: 'pick_object ' + req.body.object_id
    };
    console.log(`Sending: ${typeof request}`, request);
    changeStateClient.sendRequest(request, (response) => {
      console.log('Request sent. Response: ' + response)
    });
  });
});

app.get('/release_object', (req, res) => {
  changeStateClient.waitForService(100).then((result) => {
    if (!result) {
      console.log('Error: service not available');
      return;
    }
    console.log(req.body)
    const request = {
      state_name: 'release'
    };
    console.log(`Sending: ${typeof request}`, request);
    changeStateClient.sendRequest(request, (response) => {
      console.log('Request sent. Response: ' + response)
    });
  });
});



server.listen(port);
console.log('Express started on port ' + port);
