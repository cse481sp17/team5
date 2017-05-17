(function(document) {
    'use strict';

    var namespace = this;

    Loader.prototype = Object.create(Object.prototype);
    function Loader() {
        this._$loader = $('#loader');
        this._$text = $('#loader-text');
        return this;
    }
    Loader.prototype.constructor = Loader;

    Loader.prototype.toggle = function() {

    }
    namespace.Loader = Loader;


    Application.prototype = Object.create(Object.prototype);
    function Application() {
        this._loader = new Loader();
        return this;
    }
    Application.prototype.constructor = Application;

    Application.prototype.init = function() {
    }

    namespace.Application = Application;


}.apply(
    window.Micheal = window.Micheal ? window.Micheal : {},     // Namespace
    [window.document]   // Dependencies
));