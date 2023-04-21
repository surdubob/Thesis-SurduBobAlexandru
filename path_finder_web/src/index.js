const rclnodejs = require('rclnodejs');
const express = require('express');
const path = require('path');
var url = require('url');

const app = express();
const port = 8080;
const motor_speed = 60;

rclnodejs.init();
const node = rclnodejs.createNode('MyNode');
const publisher = node.createPublisher('std_msgs/msg/String', '/web_movement_commands');

app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'static/index.html'));
});

app.get('/movement-command/:arg', (req, res) => {
  console.log(req.params.arg);
  switch(req.params.arg) {
    case 'forward':
      publisher.publish('MB ' + motor_speed);
      break;
    case 'backward':
      publisher.publish('MB ' + -motor_speed);
      break;
    case 'left':
      publisher.publish('ML ' + -motor_speed);
      publisher.publish('MR ' + motor_speed);
      break;
    case 'right':
      publisher.publish('ML ' + motor_speed);
      publisher.publish('MR ' + -motor_speed);
      break;
    case 'diag_left':
      publisher.publish('ML ' + motor_speed / 3);
      publisher.publish('MR ' + motor_speed);
      break;
    case 'diag_right':
      publisher.publish('ML ' + motor_speed);
      publisher.publish('MR ' + motor_speed / 3);
      break;
    case 'diag_back_left':
      publisher.publish('ML ' + -motor_speed / 3);
      publisher.publish('MR ' + -motor_speed);
      break;
    case 'diag_back_right':
      publisher.publish('ML ' + -motor_speed);
      publisher.publish('MR ' + -motor_speed / 3);
      break;
    case 'stop':
      publisher.publish('MB ' + 0);
      break;
  }
  res.send(req.params.arg);
});



app.listen(port);
console.log('Express started on port ' + port);
