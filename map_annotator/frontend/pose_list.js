PoseList = function(ros) {
  // HTML elements
  var poseListDiv = document.querySelector('#poseList');
  var createButton = document.querySelector('#createButton');

  var that = this;

  var sub = new ROSLIB.Topic({
    ros: ros,
    name: '/pose_names',
    messageType: 'map_annotator/PoseNames'
  });

  // Create topic for publishing user actions to the server.
  var actionSub = new ROSLIB.Topic({
      ros: ros,
      name: '/user_actions',
      messageType: 'map_annotator/UserActions'
  });

  var render = function(poseList) {
    if (poseList.length == 0) {
      poseListDiv.textContent = "No poses."
    } else {
      poseListDiv.innerHTML = '';
      for (var i=0; i<poseList.length; ++i) {
        var pose = new Pose(ros, poseList[i], actionSub);
        var poseDiv = pose.render();
        poseListDiv.appendChild(poseDiv);
      }
    }
  }

  sub.subscribe(function(message) {
    render(message.names);
  });
  render([]);

  createButton.addEventListener('click', function() {
    var name = prompt('Enter a name for this pose:');
    if (!name) {
      return;
    }
    console.log('Creating pose with name', name);
    var message = new ROSLIB.Message();
    message.command = "create";
    message.name = name;
    window.alert("Creating Pose: " + name);
    actionSub.publish(message);
  })
}
