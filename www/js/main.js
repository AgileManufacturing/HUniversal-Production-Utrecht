"use strict";

/**
 * @Author Benno Zeeman
 */

function UIPanel() {
	$('#nav ul > li > a').click((this.navClick).bind(this));

	//this.switchSection($('#nav ul > li > a.default'));

	// Log section
	this.log_pre   = $('#log pre');
	this.log_clean = $('#log #clear');
	this.log_clean.click((this.clearLog).bind(this));


	// Grid section
	this.grid = $('#grid ul');

	this.clickAgentListener = null;


	// Controls section
	this.con_status_light = $('#controls #con-status .light');
	this.con_status_text  = $('#controls #con-status .text');

	this.btnStop  = $('#controls #stop').click((this.clickBtnStop).bind(this));
	this.btnStart = $('#controls #start').click((this.clickBtnStart).bind(this));

	this.clickBtnStopListener  = null;
	this.clickBtnStartListener = null;

	this.setConStatus(this.CLOSED);


	// Agents section
	this.agents = {};
}

UIPanel.prototype = {
	'CLOSED':      0,
	'CONNECTING':  1,
	'OPEN':        2,
	'println': function(msg, id) {
		// If argument(s) not defined, give default
		if (typeof id === 'undefined') {
			id = '';
		}

		// Collect date variables
		var date = new Date();
		var hh = date.getHours();
		var mm = date.getMinutes();
		var ss = date.getSeconds();
		var cc = date.getMilliseconds();

		// Prepend to make sure there are 2 (and 3 ms) characters
		if (hh < 10)  { hh = '0' + hh; }
		if (mm < 10)  { mm = '0' + mm; }
		if (ss < 10)  { ss = '0' + ss; }
		if (cc < 100) { cc = ('00' + cc).slice(-3); }

		// Time format, e.g. 14:15:16.12 (hh:mm:ss.ms)
		var time_str = hh + ':' + mm + ':' + ss + '.' + cc;

		// If scrolled to bottom, remember and scroll down after new content
		var bottom = this.log_pre.prop('scrollHeight') - this.log_pre.scrollTop() == this.log_pre.innerHeight();


		var $time = $('<em></em>',     {'class': 'time'}).text(time_str);
		var $id   = $('<em></em>',     {'class': 'id'}).text(id);
		var $msg  = $('<span></span>', {'class': 'msg'}).text(msg);

		var $container = $('<div></div>', {'class': 'line'});
		$container
			.append($time)
			.append($id)
			.append($msg)
		;

		// Write the log message in certain format
		this.log_pre.append($container);

		// Scroll down for 'sticky' effect.
		if (bottom === true) {
			this.log_pre.scrollTop(this.log_pre.prop('scrollHeight'));
		}
	},
	'addAgent': function(id, type, state) {
		var $li, $a;

		if (id in this.agents) {
			this.println('Updating agent "' + id + '"', 'ui');
			$a = this.agents[id].$a;
		} else {
			this.println('Adding new agent "' + id + '"', 'ui');

			// Create UI elements with attributes
			$li = $('<li></li>', {'data-id': id, 'data-type': type, 'class': type});
			$a  = $('<a></a>', {'href': '#', on: {click: (this.clickAgent).bind(this)}});

			$li.append($a);

			// Insert into UI
			this.grid.append($li);

			this.agents[id] = { $a: $a, $li: $li };
		}

		$a.text(id + ':' + state);
	},
	'clearLog': function(event) {
		if (typeof(event) !== 'undefined') {
			event.preventDefault();
		}

		this.log_pre.empty();
	},

	'setConStatus': function(con_status) {
		if (con_status == this.CLOSED) {
			this.con_status_light.css('background-color', 'red');
			this.con_status_text.text('Closed');
		} else if (con_status == this.CONNECTING) {
			this.con_status_light.css('background-color', 'yellow');
			this.con_status_text.text('Connecting');
		} else if (con_status == this.OPEN) {
			this.con_status_light.css('background-color', 'green');
			this.con_status_text.text('Open');
		}
	},

	'clickAgent': function(event) {
		event.preventDefault();

		// Retreive elements
		var $target = $(event.target);
		var $parent = $target.parent();
		var id      = $parent.data('id');

		this.println('Clicked agent "' + id + '"', 'ui');

		if (this.clickAgentListener !== null) {
			this.clickAgentListener(id);
		}
	},
	'setClickAgentListener': function(listener) { this.clickAgentListener = listener; },


	'clickBtnStop': function(event) {
		this.btnStop.prop('disabled', true);
		this.btnStart.prop('disabled', false);

		console.log(this.btnStart);

		if (this.clickBtnStopListener !== null) {
			this.clickBtnStopListener();
		}
	},
	'setClickBtnStopListener': function(listener) { this.clickBtnStopListener = listener; },

	'clickBtnStart': function(event) {
		this.btnStop.prop('disabled', false);
		this.btnStart.prop('disabled', true);

		if (this.clickBtnStartListener !== null) {
			this.clickBtnStartListener();
		}
	},
	'setClickBtnStartListener': function(listener) { this.clickBtnStartListener = listener; },

	'navClick': function(event) {
		event.preventDefault();

		var $a = $(event.target);

		this.switchSection($a);
	},
	'switchSection': function($a) {
		$('#nav ul > li > a').removeClass('current');
		$a.addClass('current');

		$('main > section').hide();
		$('main > section#' + $a.data('section')).show();
	},

	'clean': function() {
		$('#grid ul').html('');

		this.agents = {};
	},
};

// Constructor of Server class
function Server(host, port, id) {
	this.id  = id;
	this.url = host + ':' + port;

	// When true will attempt reconnecting when disconnect
	this.will_reconnect = true;
	// Connection status
	this.is_connected    = false;
	// The WebSocket instance
	this.websocket    = null;

	// The UI object used to let the world know what the f*ck we're doing
	this.ui = new UIPanel();
	this.ui.setClickAgentListener((this.requestAgent).bind(this));
	this.ui.setClickBtnStopListener((this.disconnect).bind(this));
	this.ui.setClickBtnStartListener((this.connect).bind(this));

	this.ui.println('Server("' + host + '", ' + port + ')', this.id);
}

// Server methods
Server.prototype = {
	'onerror': function(event) {
		if (!this.is_connected) {
			return;
		}
		this.ui.println('WebSocket error', this.id);
		//console.log('onerror (event next)');
		//console.log(event);
	},
	'onmessage': function(event) {
		var data = JSON.parse(event.data);

		// Switch what type of message is received
		// Act accordingly
		switch (data['command']) {
			case 'GET_OVERVIEW':
				this.ui.println('Received GETOVERVIEW', this.id);

			 	for(var i = 0; i < data['agents'].length; i++) {
 					var agent = data['agents'][i];
					this.ui.addAgent(agent['id'], agent['type'], agent['state']);
 				}

				break;
			case 'ADDAGENT':
				this.ui.println('Received ADDAGENT', this.id);

 				var agent = data['agent'];
				this.ui.addAgent(agent['id'], agent['type'], agent['state']);

				break;
			case 'ON_EQUIPLET_STATE_CHANGED':
				this.ui.println('Received ON_EQUIPLET_STATE_CHANGED', this.id);

 				var agent = data['agent'];
				this.ui.addAgent(agent['id'], null, agent['state']);

				break;
			default:
				this.ui.println('Received unknown "' + data['command'] + '"', this.id);
				break;
		}
	},
	'onopen': function(event) {
		this.ui.println('Connected to web socket', this.id);

		// Clear potential timeout trying to reconnect
		clearTimeout(this.timeout);

		this.ui.setConStatus(this.ui.OPEN);
		this.is_connected = true;

		// Request grid overview from WS
		this.send('GETOVERVIEW');
	},
	'onclose': function(event) {
		// Find respective descriptions for error codes
		// https://tools.ietf.org/html/rfc6455#section-11.7
		switch (event.code) {
			case 1000: var desc = 'Normal Closure';             break;
			case 1001: var desc = 'Going Away';                 break;
			case 1002: var desc = 'Protocol error';             break;
			case 1003: var desc = 'Unsupported Data';           break;
			case 1005: var desc = 'No Status Rcvd';             break;
			case 1006: var desc = 'Abnormal Closure';           break;
			case 1007: var desc = 'Invalid frame payload data'; break;
			case 1008: var desc = 'Policy Violation';           break;
			case 1009: var desc = 'Message Too Big';            break;
			case 1010: var desc = 'Mandatory Ext.';             break;
			case 1011: var desc = 'Internal Server Error';      break;
			case 1015: var desc = 'TLS handshake';              break;
			default:   var desc = 'Unrecognized error code';    break;
		}

		//console.log('onclose (event next)');
		//console.log(event);

		// If is_connected is false, connection was closed purposedly
		if (this.is_connected) {
			this.ui.println('Connection closed with "' + desc + '"', this.id);
			this.is_connected = false;
		}

		this.ui.setConStatus(this.ui.CLOSED);
		this.websocket    = null;

		if (this.will_reconnect) {
			this.timeout = setTimeout((this.connect).bind(this), 3000);
		}
	},

	'connect': function() {
		this.ui.println('Attempting to connect to "' + this.url + '"', this.id);
		this.ui.setConStatus(this.ui.CONNECTING);

		this.will_reconnect = true;

		this.websocket = new WebSocket('ws://' + this.url);

		// Callbacks with correct context (this)
		this.websocket.onerror   = (this.onerror).bind(this);
		this.websocket.onmessage = (this.onmessage).bind(this);
		this.websocket.onclose   = (this.onclose).bind(this);
		this.websocket.onopen    = (this.onopen).bind(this);

		return this.websocket;
	},

	'disconnect': function() {
		this.ui.println('Stopping client', this.id);
		this.ui.setConStatus(this.ui.CLOSED);

		this.ui.clean();

		this.is_connected      = false;
		this.will_reconnect = false;
		if (this.timeout !== null) {
			clearTimeout(this.timeout);
		}

		//if (this.websocket !== null) {
			this.websocket.close();
		//	this.websocket = null;
		//}
	},

	// Send to WS
	'send': function(command, id, values) {
		if (typeof(id)     === 'undefined') { id     = null; }
		if (typeof(values) === 'undefined') { values = []; }

		// Create JSON message to send over WS
		var str = JSON.stringify({
			'command': command,
			'id':      id,
			'values':  values,
		});
		this.websocket.send(str);
	},

	'toJson': function() {

	},

	'requestAgent': function(id) {
		if (!this.is_connected) {
			return;
		}

		this.ui.println('Get Agent "' + id + '" information from SCADA', this.id);

		this.send('GET_AGENT_INFO', id);
	},
};

// Server class allowing us to connect to web socket.
var server = new Server(window.location.hostname, 3529, 'SCADA');


//server.connect();

/*window.addEventListener('beforeunload', function (e) {
	var msg = 'Leaving causes acquired data to be discarded';

	(e || window.event).returnValue = msg;
    return msg;
});*/
