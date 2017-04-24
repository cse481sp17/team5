(function (document) {
    'use strict';

    var namespace = this;

    /**
     * UserActionMessage is the message object type for
     * topic /user_actions
     */
    UserActionMessage.prototype = Object.create(namespace.Message.prototype);

    /**
     * Command definition types for the topic.
     */
    UserActionMessage.prototype.CommandDefinition = {
        CREATE: 'create',
        DELETE: 'delete',
        GOTO: 'goto',
        RENAME: 'rename' // UNIMPLEMENTED
    }

    /**
     * Create a new instance of the UserActionMessage class.
     */
    function UserActionMessage() {
        this.command = '';
        this.name = '';
        this.update_name = '';
        return this;
    }
    UserActionMessage.prototype.constructor = UserActionMessage;

    // Export to Namespace.
    namespace.UserActionMessage = UserActionMessage;

    /**
     * UserActionController controls user interactions with poses.
     */
    UserActionController.prototype = Object.create(Object.prototype);

    /**
     * Construct a new instance of the UserActionController class.
     * @param {*} ros The ROS instance connection.
     * @param {*} container The JQuery object container for rendering.
     */
    function UserActionController(ros, $container) {

        // Create topic for publishing user actions to the server.
        this._sub = new namespace.Topic({
            ros: ros,
            name: '/user_actions',
            messageType: 'map_annotator/UserActions'
        });

        // Initialize the client view and bind listeners.
        this._initialize($container);

        // Create the topic for subscribing to poses.
        this._poseSub = new namespace.Topic({
            ros: ros,
            name: '/pose_names',
            messageType: 'map_annotator/PoseNames'
        });

        // Subscribe to changes in the pose set.
        this._poseSub.subscribe = this._handlePoseSubscription;

        return this;
    }
    UserActionController.prototype.constructor = UserActionController;

    /**
     * Handle a subscription notification from the pose node.
     * NOTE: Rendering must occur prior to the first subscription notification.
     */
    UserActionController.prototype._handlePoseSubscription = function (message) {
        this._$poseSelection.find('option').remove();

        var $poseSelection = this._$poseSelection;
        $.each(message.names, function (i, name) {
            $poseSelection.append($('<option/>', {
                value: name,
                text: name
            }));
        });
    }

    /**
     * Initialize and render the client view.
     */
    UserActionController.prototype._initialize = function ($container) {
        var self = this;
        this._$poseSelection = $('<select>')
            .attr({ multiple: "yes" });
        this._$poseSelection.append($('<option/>', {
            value: "Option",
            text: "Option"
        }));

        // Button for creating new poses.
        this._$createButton = $('<input/>')
            .attr({
                type: "button",
                value: "Create"
            })
            .click(function () {
                var poseName = window.prompt("Pose Name?");
                if (poseName == null || poseName == "") {
                    // Canceled input.
                    return;
                }

                var message = new namespace.UserActionMessage();
                message.command = namespace.UserActionMessage.prototype.CREATE;
                message.name = poseName;

                window.alert("Creating Pose: " + poseName);
                self._sub.publish(message);
            });

        // Button for deleting existing poses.
        this._$deleteButton = $('<input/>')
            .attr({
                type: "button",
                value: "Delete"
            })
            .click(function () {
                var poseName = self._$poseSelection.val();
                if (poseName == null || poseName == "") {
                    // No option selected.
                    window.alert("Please select a Pose.");
                    return;
                }

                var message = new namespace.UserActionMessage();
                message.command = namespace.UserActionMessage.prototype.DELETE;
                message.name = poseName;
                window.alert("Deleting Pose: " + poseName);
                self._sub.publish(message);
            });

        // Button for going to existing poses.
        this._$gotoButton = $('<input/>')
            .attr({
                type: "button",
                value: "GoTo"
            })
            .click(function () {
                var poseName = self._$poseSelection.val();
                if (poseName == null || poseName == "") {
                    // No option selected.
                    window.alert("Please select a Pose.");
                    return;
                }
                var message = new namespace.UserActionMessage();
                message.command = namespace.UserActionMessage.prototype.GOTO;
                message.name = poseName;
                window.alert("Going to Pose: " + poseName);
                self._sub.publish(message);
            });

        $container.append(this._$poseSelection);
        $container.append(this._$createButton);
        $container.append(this._$deleteButton);
        $container.append(this._$gotoButton);

        this._$container = $container;

        return this;
    }

    // Export to namespace
    namespace.UserActionController = UserActionController;

}.apply(window.ROSLIB, // Namespace
    [window.document] // Dependencies
    ));

