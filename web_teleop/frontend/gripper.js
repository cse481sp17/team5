Gripper = function(ros) {
  // HTML elements
//   var gripperState = document.querySelector('#gripperState');
  var grip_openButton = document.querySelector('#grip_openButton');
  var grip_CloseButton = document.querySelector('#grip_CloseButton');

  var that = this;

  var setGripper = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_gripper',
    serviceType: 'web_teleop/SetGripper'
  });

  grip_openButton.addEventListener('click', function() {
    var request = new ROSLIB.ServiceRequest({
      position: 0.1
    });
    setGripper.callService(request);
  });

  grip_CloseButton.addEventListener('click', function() {
    var request = new ROSLIB.ServiceRequest({
      position: 0.0
    });
    setGripper.callService(request);
  });

}