Pose = function(ros, name) {
  var that = this;
  this.name = name;

  var actionPublisher = new ROSLIB.Topic({
      ros: ros,
      name: '/user_actions',
      messageType: 'map_annotator/UserAction'
  });

  function handleGoTo() {
    console.log('GoTo ' + name + ' clicked.');
    var message = new ROSLIB.Message();
    message.command = "goto";
    message.name = name;
    actionPublisher.publish(message);
  }

  function handleDelete() {
    console.log('Delete ' + name + ' clicked.');
    var message = new ROSLIB.Message();
    message.command = "delete";
    message.name = name;
    actionPublisher.publish(message);
  }

  this.render = function() {
    var node = document.createElement('div');
    var nameNode = document.createTextNode(name);
    node.appendChild(nameNode);

    var sendNode = document.createElement('input');
    sendNode.type = 'button';
    sendNode.value = 'Go to';
    sendNode.addEventListener('click', handleGoTo);
    node.appendChild(sendNode);

    var deleteNode = document.createElement('input');
    deleteNode.type = 'button';
    deleteNode.value = 'Delete';
    deleteNode.addEventListener('click', handleDelete);
    node.appendChild(deleteNode);
    return node;
  }
}
