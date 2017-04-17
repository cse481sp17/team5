Arm = function(ros) {
  // HTML elements
  var armShoulderPan = document.querySelector('#armShoulderPan');
  var armShoulderLift = document.querySelector('#armShoulderLift');
  var armUpperarmRoll = document.querySelector('#armUpperarmRoll');
  var armElbowFlex = document.querySelector('#armElbowFlex');
  var armForearmRoll = document.querySelector('#armForearmRoll');
  var armWristFlex = document.querySelector('#armWristFlex');
  var armWristRoll = document.querySelector('#armWristRoll');
  
  var desiredArmShoulderPan = document.querySelector('#desiredArmShoulderPan');
  var desiredArmShoulderLift = document.querySelector('#desiredArmShoulderLift');
  var desiredArmUpperarmRoll = document.querySelector('#desiredArmUpperarmRoll');
  var desiredArmElbowFlex = document.querySelector('#desiredArmElbowFlex');
  var desiredArmForearmRoll = document.querySelector('#desiredArmForearmRoll');
  var desiredArmWristFlex = document.querySelector('#desiredArmWristFlex');
  var desiredArmWristRoll = document.querySelector('#desiredArmWristRoll');

  var armShoulderPanSlider = document.querySelector('#armShoulderPanSlider');
  var armShoulderLiftSlider = document.querySelector('#armShoulderLiftSlider');
  var armUpperarmRollSlider = document.querySelector('#armUpperarmRollSlider');
  var armElbowFlexSlider = document.querySelector('#armElbowFlexSlider');
  var armForearmRollSlider = document.querySelector('#armForearmRollSlider');
  var armWristFlexSlider = document.querySelector('#armWristFlexSlider');
  var armWristRollSlider = document.querySelector('#armWristRollSlider');

  var armButton = document.querySelector('#armButton');

  var that = this;

  var setArmClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_arm',
    serviceType: 'web_teleop/SetArm'
  });

  //
  var armShoulderPanListener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/shoulder_pan_joint',
    messageType: 'std_msgs/Float64'
  });

  armShoulderPanListener.subscribe(function(message) {
    var position = message.data;
    position = Math.round(position*1000) / 1000
    armShoulderPan.textContent = position;
  });

  //
  var armShoulderLiftListener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/shoulder_lift_joint',
    messageType: 'std_msgs/Float64'
  });

  armShoulderLiftListener.subscribe(function(message) {
    var position = message.data;
    position = Math.round(position*1000) / 1000
    armShoulderLift.textContent = position;
  });

  //
  var armUpperarmRollListener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/upperarm_roll_joint',
    messageType: 'std_msgs/Float64'
  });

  armUpperarmRollListener.subscribe(function(message) {
    var position = message.data;
    position = Math.round(position*1000) / 1000
    armUpperarmRoll.textContent = position;
  });

  //
  var armElbowFlexListener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/elbow_flex_joint',
    messageType: 'std_msgs/Float64'
  });

  armElbowFlexListener.subscribe(function(message) {
    var position = message.data;
    position = Math.round(position*1000) / 1000
    armElbowFlex.textContent = position;
  });

  //
  var armForearmRollListener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/forearm_roll_joint',
    messageType: 'std_msgs/Float64'
  });

  armForearmRollListener.subscribe(function(message) {
    var position = message.data;
    position = Math.round(position*1000) / 1000
    armForearmRoll.textContent = position;
  });

  //
  var armWristFlexListener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/wrist_flex_joint',
    messageType: 'std_msgs/Float64'
  });

  armWristFlexListener.subscribe(function(message) {
    var position = message.data;
    position = Math.round(position*1000) / 1000
    armWristFlex.textContent = position;
  });

  //
  var armWristRollListener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/wrist_roll_joint',
    messageType: 'std_msgs/Float64'
  });
  
  armWristRollListener.subscribe(function(message) {
    // Whenever we get a message with a new torso height, update
    // the torso height display on the webpage.
    var position = message.data;
    // Note the noise in the data. You can smooth it out using this line of code.
    position = Math.round(position*1000) / 1000
    armWristRoll.textContent = position;
  });
  

  // Initialize slider.
  //var desiredPositions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
  var desiredPositions0 = 0.0;
  var desiredPositions1 = 0.0;
  var desiredPositions2 = 0.0;
  var desiredPositions3 = 0.0;
  var desiredPositions4 = 0.0;
  var desiredPositions5 = 0.0;
  var desiredPositions6 = 0.0;
  desiredArmShoulderPan.textContent = desiredPositions0;
  desiredArmShoulderLift.textContent = desiredPositions1;
  desiredArmUpperarmRoll.textContent = desiredPositions2;
  desiredArmElbowFlex.textContent = desiredPositions3;
  desiredArmForearmRoll.textContent = desiredPositions4;
  desiredArmWristFlex.textContent = desiredPositions5;
  desiredArmWristRoll.textContent = desiredPositions6;

  armShoulderPanSlider.value = desiredPositions0;
  armShoulderLiftSlider.value = desiredPositions1;
  armUpperarmRollSlider.value = desiredPositions2;
  armElbowFlexSlider.value = desiredPositions3;
  armForearmRollSlider.value = desiredPositions4;
  armWristFlexSlider.value = desiredPositions5;
  armWristRollSlider.value = desiredPositions6;


  // Update  when slider moves.
  armShoulderPanSlider.addEventListener('input', function() {
    desiredPositions0 = armShoulderPanSlider.value;
    desiredArmShoulderPan.textContent = desiredPositions0;
  });

  // Update  when slider moves.
  armShoulderLiftSlider.addEventListener('input', function() {
    desiredPositions1 = armShoulderLiftSlider.value;
    desiredArmShoulderLift.textContent = desiredPositions1;
  });

  // Update  when slider moves.
  armUpperarmRollSlider.addEventListener('input', function() {
    desiredPositions2 = armUpperarmRollSlider.value;
    desiredArmUpperarmRoll.textContent = desiredPositions2;
  });

  // Update  when slider moves.
  armElbowFlexSlider.addEventListener('input', function() {
    desiredPositions3 = armElbowFlexSlider.value;
    desiredArmElbowFlex.textContent = desiredPositions3;
  });

  // Update  when slider moves.
  armForearmRollSlider.addEventListener('input', function() {
    desiredPositions4 = armForearmRollSlider.value;
    desiredArmForearmRoll.textContent = desiredPositions4;
  });

  // Update  when slider moves.
  armWristFlexSlider.addEventListener('input', function() {
    desiredPositions5 = armWristFlexSlider.value;
    desiredArmWristFlex.textContent = desiredPositions5;
  });

  // Update  when slider moves.
  armWristRollSlider.addEventListener('input', function() {
    desiredPositions6 = armWristRollSlider.value;
    desiredArmWristRoll.textContent = desiredPositions6;
  });
  

  // Method to set the height.
  this.setPositions = function(position0, position1, position2, position3, position4, position5, position6) {
    // do not know the range of variable in the position
    //var height = Math.min(Math.max(0.0, height), 0.4);
    var position0 = -Math.min(Math.max(-1.6, position0), 1.6);
    var position1 = -Math.min(Math.max(-1.2, position1), 1.5);
    var position2 = Math.min(Math.max(-3.0, position2), 3.0);
    var position3 = -Math.min(Math.max(-2.25, position3), 2.25);
    var position4 = Math.min(Math.max(-3.0, position4), 3.0);
    var position5 = -Math.min(Math.max(-2.15, position5), 2.15);
    var position6 = Math.min(Math.max(-3.0, position6), 3.0);

    var request = new ROSLIB.ServiceRequest({
      armShoulderPan: position0,
      armShoulderLift: position1,
      armUpperarmRoll: position2,
      armElbowFlex: position3, 
      armForearmRoll: position4,
      armWristFlex: position5,
      armWristRoll: position6
    });
    setArmClient.callService(request);
  };



  // Set the height when the button is clicked.
  armButton.addEventListener('click', function() {
    that.setPositions(desiredPositions0, desiredPositions1, desiredPositions2, desiredPositions3, desiredPositions4, desiredPositions5, desiredPositions6)
  });
}