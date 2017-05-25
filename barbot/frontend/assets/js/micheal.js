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
        this._$order.deferredFadeOut()
        .then(() => {self._loader._$text.text('Creating Drink.'); return self._loader.show();})
        .then(() => self._sendOrder(type, ammount))
        .then(
            () => self._loader.setText('Drink Complete.'),
            () => self._loader.setText('Drink Failed.'))
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
        ros.on('connection', function() {
            self._drinkService = new ROSLIB.Service({
                ros: ros,
                name: '/user_actions',
                serviceType : 'UserAction'
            });
            p.resolve();
        });
        ros.on('error', function(error) {
            p.reject(error);
        });
        ros.on('close', function() {
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
        var p = $.Deferred();
        var guid = generateGuid();
        var request = new ROSLIB.ServiceRequest({
            command : 'make_drink',
            type : type,
            id : guid
        });
        console.info("Making drink service call.");
        this._drinkService.callService(
            request,
            (result) => {
                console.info("Service call returned.");
                p.resolve();
        });

        return p.promise();
    };

    // Export to namespace.
    namespace.Application = Application;

}.apply(
    window.Micheal = window.Micheal ? window.Micheal : {}, // Namespace
    [window.jQuery, window.document, window.ROSLIB, window.Materialize]   // Dependencies
));