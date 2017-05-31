(function($, document, ROSLIB, Materialize) {
    'use strict';

    /**
     * Extend JQuery Fade to support asynchrounous
     * fluency.
     */
    $.fn.extend({
        deferredFadeOut: function(speed) {
            var p = $.Deferred();
            this.fadeOut(speed, function() {p.resolve();});
            return p.promise();
        },
        deferredFadeIn: function(speed) {
            var p = $.Deferred();
            this.fadeIn(speed, function() {p.resolve();});
            return p.promise();
        },
        deferredAnimate: function(property, speed) {
            var p = $.Deferred();
            this.animate(property, speed, () => p.resolve());
            return p.promise();
        }
    });

    /**
     * Generate a random GUID per the RFC4122 version 4 specification.
     * Should we be doing this client side. No. But the alternative adds another
     * node of complexity.
     */
    var generateGuid = function() {
        var d = new Date().getTime();
        if (typeof performance !== 'undefined' && typeof performance.now === 'function'){
            d += performance.now(); //use high-precision timer if available
        }
        return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function (c) {
            var r = (d + Math.random() * 16) % 16 | 0;
            d = Math.floor(d / 16);
            return (c === 'x' ? r : (r & 0x3 | 0x8)).toString(16);
        });
    };

    /**
     * Anonymously reject a promise.
     */
    var reject = function() {
        return $.Deferred().reject();
    };

    var namespace = this;

    /**
     * Loader is a representation of the loading screen overlay.
     */
    Loader.prototype = Object.create(Object.prototype);

    /**
     * Construct a new Loader instance.
     */
    function Loader() {
        this._$loader = $('#loader');
        this._$text = $('#loader-text');
        this.fadeSpeed = 400;
        return this;
    }
    Loader.prototype.constructor = Loader;

    /**
     * Hide the loader.
     * @return A promise to fade out the loader.
     */
    Loader.prototype.hide = function() {
        return this._$loader.deferredFadeOut(this.fadeSpeed);
    };

    /**
     * Show the loader.
     * @return A promise to fade in the loader.
     */
    Loader.prototype.show = function() {
        return this._$loader.deferredFadeIn(this.fadeSpeed);
    };

    /**
     * Set the text of the loader.
     * @param text The text to set the loader to.
     * @return A promise to exchange the current text with the provided text.
     */
    Loader.prototype.setText = function(text) {
        var self = this;
        var $text = self._$text;
        return $text.deferredAnimate({'opacity' : 0}, self.fadeSpeed)
        .then(() => {$text.text(text); return $text.deferredAnimate({'opacity': 1}, self.fadeSpeed * 2); });
    };

    // Export to namespace.
    namespace.Loader = Loader;

    /**
     * The Micheal BarBot Application.
     */
    Application.prototype = Object.create(Object.prototype);

    /**
     * Construct a new Applicaton instance. 
     */
    function Application() {
        this._loader = new Loader();
        this._$order = $('#order');
        this._toastSpeed = 5000;
        this._deferredOrder = null;
        this._deferredOrderId = null;
        return this;
    };
    Application.prototype.constructor = Application;

    /**
     * Initialize the application.
     * @param url The url of the Fetch Robot.
     */
    Application.prototype.init = function(url) {
        this._url = url ? url : 'ws://robonaut.cs.washington.edu:9090';
        var self = this;
        Materialize.toast('Using ' + this._url, self._toastSpeed);
        this._connect(this._url)
        .then(
            () => self._loader.setText('Connection Successfull.'),
            () => self._loader.setText('Connection Failed.').then(() => reject()))
        .then(
            () => self._loader.hide(),
            () => reject())
        .then(
            () => self._$order.deferredFadeIn(),
            () => reject());
    };

    /**
     * Order a drink from Micheal.
     * @param type The type of drink to order.
     * @param ammount The ammount of drink to order (oz).
     */
    Application.prototype.order = function(type, ammount) {
        var self = this;
        Materialize.toast('Ordered Drink: ' + type + ' ' + ammount + 'oz', self._toastSpeed);
        var interval;
        self._$order.deferredFadeOut()
        .then(() => {
            self._loader._$text.html('Creating Drink. <h4 style="text-align: center" id="timer">0m 00s</h4>');
            var time = 0;
            var $timer = $('#timer');
            interval = window.setInterval(() => {
		time++;
		var minutes = Math.floor(time/60);
		var seconds = time % 60;
		$timer.text(minutes + "m " + ('0' + seconds).slice(-2) + "s");
		}, 1000);
            return self._loader.show();})
        .then(() => self._sendOrder(type, ammount))
        .then(
            () => {
                window.clearInterval(interval);
                Materialize.toast('Drink Completed.', self._toastSpeed);
                Materialize.toast('Enjoy!', self._toastSpeed);
                return self._loader.setText('Drink Complete.')
                .then(() => self._$order.deferredFadeIn());
            }
            ,
            (error) => {
                window.clearInterval(interval);
                Materialize.toast('Error: ' + error.type, self._toastSpeed);
                return self._loader.setText('Drink Failed.');
            })
        .then(() => self._loader.hide());
    };

    /**
     * Connect the application to the Fetch Robot.
     * @param url The url of the robot.
     * @return A promise to connect to the robot.
     */
    Application.prototype._connect = function(url) {
        var self = this;
        var p = $.Deferred();
        var ros = new ROSLIB.Ros({url: url});
        ros.on('connection', () => {
            Materialize.toast('Websocket connection opened.', self._toastSpeed);
            self._drinkPublisher = new ROSLIB.Topic({
                ros: ros,
                name: '/drink_order',
                messageType: 'barbot/DrinkOrder'
             });
             self._drinkSubscriber = new ROSLIB.Topic({
                ros: ros,
                name: '/drink_status',
                messageType: 'barbot/DrinkStatus',
                throttle_rate: 10,
            });
            self._drinkSubscriber.subscribe(
                /** msg.DrinkStatus */
                function(message) {
                    console.info("Updated received.");
                    // Whenever a drink status is received we need to see if a drink was complete
                    //  and if that completed drink was requested by us.
                    if(message.completed !== undefined && self._deferredOrderId == message.completed) {
                        self._deferredOrder.resolve();
                        self._deferredOrder = null;
                        self._deferredOrderId = null;
                    } else {
                        for(var i = 0; i < message.orders.length; i++) {
                            if(message.orders[i] == self._deferredOrderId) {
                                 Materialize.toast('Currently in queue: ' + (i + 1) + ' of ' + message.orders.length, self._toastSpeed);
                                 break;
                            }
                        }
                    }
                });
            p.resolve();
        });
        ros.on('error', (error) => {
            Materialize.toast('Error: ' + error.type, self._toastSpeed);
            p.reject(error);
        });
        ros.on('close', () => {
            Materialize.toast('Websocket connection closed.', self._toastSpeed);
            p.reject();
        });
        this._ros = ros;
        return p.promise();
    };

    /**
     * Send a drink order to the robot.
     * @param type The type of drink.
     * @param ammount The ammount of drink (oz).
     */
    Application.prototype._sendOrder = function(type, ammount) {
        var self = this;
        var p = $.Deferred();
        var guid = generateGuid();

        /** msg.DrinkOrder */
        var message = new ROSLIB.Message({
            command: 'make_order',
            id: guid,
            type: type,
            ammount: String(ammount)
        });

        self._deferredOrder = p;
        self._deferredOrderId = guid;
        self._drinkPublisher.publish(message);

        console.info("Drink order published: " + guid);

        return p.promise();
    };

    // Export to namespace.
    namespace.Application = Application;

}.apply(
    window.Micheal = window.Micheal ? window.Micheal : {}, // Namespace
    [window.jQuery, window.document, window.ROSLIB, window.Materialize]   // Dependencies
));
