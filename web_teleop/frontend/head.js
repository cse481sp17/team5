Head = function(ros) {
  // HTML elements
  var panAngle = document.querySelector('#panAngle');
  var desiredPanAngle = document.querySelector('#desiredPanAngle');
  var panSlider = document.querySelector('#panSlider');
  var tiltAngle = document.querySelector('#tiltAngle');
  var desiredTiltAngle = document.querySelector('#desiredTiltAngle');
  var tiltSlider = document.querySelector('#tiltSlider');
  var pan_tiltbtn = document.querySelector('#pan_tiltbtn');

  var that = this;

  var setHeadPanTiltClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_head_pan_tilt',
    serviceType: 'web_teleop/SetHeadPanTilt'
  });

  // Listen to head pan from the joint_state_republisher.
  var head_pan_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/head_pan_joint',
    messageType: 'std_msgs/Float64'
  });

  var head_tilt_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/head_tilt_joint',
    messageType: 'std_msgs/Float64'
  });

  head_pan_listener.subscribe(function(message) {
    // Whenever we get a message with a new torso height, update
    // the torso height display on the webpage.
    var angle = message.data;
    // Note the noise in the data. You can smooth it out using this line of code.
    angle = Math.round(angle*1000) / 1000
    panAngle.textContent = angle;
  });

  head_tilt_listener.subscribe(function(message) {

    var angle = message.data;
    // Note the noise in the data. You can smooth it out using this line of code.
    angle = Math.round(angle*1000) / 1000
    tiltAngle.textContent = angle;
  });
  
  // Initialize slider.
  var desiredPan = 0.0;
  desiredTiltAngle.textContent = desiredPan;
  var desiredTilt = 0.0;
  desiredTiltAngle.textContent = desiredTilt;
  // For most input elements, the .value field is both a getter and a setter.
  // Here we can set its value to the default (0.0).
  panSlider.value = desiredPan;
  tiltSlider.value = desiredTilt;

  // Update desiredHeight when slider moves.
  panSlider.addEventListener('input', function() {
    // Read where the slider is now.
    desiredPan = panSlider.value;
    // Update the desired torso height display.
    desiredPanAngle.textContent = desiredPan;
  });

  tiltSlider.addEventListener('input', function() {
    // Read where the slider is now.
    desiredTilt = tiltSlider.value;
    // Update the desired torso height display.
    desiredTiltAngle.textContent = desiredTilt;
  });

  

  // Method to set the height.
  this.setPanTilt = function(pan, tilt) {
    var pan = Math.min(Math.max(-1.5708, pan), 1.5708);
    var tilt = Math.min(Math.max(-0.785398, tilt), 1.5708);
    var request = new ROSLIB.ServiceRequest({
      pan: pan,
      tilt: tilt
    });
    setHeadPanTiltClient.callService(request);
  };

  // Set the height when the button is clicked.
  pan_tiltbtn.addEventListener('click', function() {
    that.setPanTilt(desiredPan, desiredTilt);
  });
}