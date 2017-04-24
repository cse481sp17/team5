Pose = function(ros, name, sub) {
  var that = this;
  this.name = name;

  function handleGoTo() {
    console.log('Go to ' + name + ' clicked.');
    var message = new ROSLIB.message();

    message.command = "goto";
    message.name = name;
    sub.publish(message);
  }

  function handleDelete() {
    console.log('Delete ' + name + ' clicked.');
    message.command = "delete";
    message.name = name;
    sub.publish(message);
  }

  this.render = function() {
    var node = document.createElement('div');
    var nameNode = document.createTextNode(name);
    node.appendChild(nameNode)

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
