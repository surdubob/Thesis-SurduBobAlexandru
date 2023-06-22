const rclnodejs = require('rclnodejs');
const express = require('express');
const path = require('path');
var url = require('url');
const app = express();
const server = require('http').Server(app)
const io = require('socket.io')(server)
const port = 8080;
const motor_speed = 60;

// rclnodejs.init();
// const node = rclnodejs.createNode('MyNode');
// const publisher = node.createPublisher('std_msgs/msg/String', '/web_movement_commands');

var last_map = null;
var last_pose = null;
var last_objects = null;

var ros_node = null;
var goal_publisher = null;

rclnodejs.init().then(() => {
  ros_node = new rclnodejs.Node('MyNode');

  ros_node.createSubscription('nav_msgs/msg/OccupancyGrid', '/map', (msg) => {
    // console.log(`Received message: ${typeof msg}`, msg);
    last_map = msg;
  });

  // ros_node.createSubscription('geometry_msgs/msg/PoseWithCovarianceStamped', '/pose', (msg) => {
  //   // console.log(`Received message: ${typeof msg}`, msg);
  //   last_pose = msg;
  // });
  ros_node.createSubscription('tf2_msgs/msg/TFMessage', '/tf', (msg) => {
    // console.log(`Received message: ${typeof msg}`, msg);
    msg.transforms.forEach(tr => {
      if (tr.header.frame_id == 'odom' && tr.child_frame_id == 'base_link'){
        last_pose = {pose: {pose: {position: tr.transform.translation, orientation: tr.transform.rotation}}}
      }
    });
  });

  ros_node.createSubscription('sensor_msgs/msg/CompressedImage', '/object_detection_image/compressed', (msg) => {
    io.emit('image', msg);
  })

  ros_node.createSubscription('visualization_msgs/msg/MarkerArray', '/detected_object_markers', (msg) => {
    last_objects = msg
  })

  publisher = ros_node.createPublisher('std_msgs/msg/Float64MultiArray', '/goal_pose');

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

app.post('/publish_goal_pose', (req, res) => {
  const secondsAndNanos = ros_node.getClock().now().secondsAndNanoseconds;
  var stamp = {
    sec: secondsAndNanos.seconds,
    nanosec: secondsAndNanos.nanoseconds,
  };
  
  // goal_publisher.publish({
  //   header: {
  //     stamp: stamp,
  //     frame_id: 'map',
  //   },
  //   pose: {
  //     position: {
  //       x: 0.0,
  //       y: 0.0,
  //       z: 0.0
  //     },
  //     orientation: {
  //       x: 0.0,
  //       y: 0.0,
  //       z: 0.0,
  //       w: 0.0
  //     }
  //   } 
  // });
  res.send('OK');
});


server.listen(port);
console.log('Express started on port ' + port);
