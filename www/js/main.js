"use strict";

/**
 * @Author Benno Zeeman
 */

function UIPanel() {
	this.log_pre   = $('#log pre');
	this.log_clean = $('#log #clear');
	this.log_clean.click((this.clearLog).bind(this));

	this.grid = $('#grid ul');

	this.con_status_light = $('#controls #con-status .light');
	this.con_status_text  = $('#controls #con-status .text');

	this.setConStatus(this.CLOSED);

	this.agentClickListener = null;
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
		this.println('Adding agent "' + id + '"', 'ui');

		// Create UI elements with attributes
		var $li = $('<li></li>', {'data': {'id': id}, 'class': type});
		var $a  = $('<a></a>',   {'href': '#', on: {click: (this.agentClick).bind(this)}});

		// Set structure
		$li.append($a);
		$a.text(id);

		// Insert into UI
		this.grid.append($li);
	},
	'agentClick': function(event) {
		event.preventDefault();

		// Retreive elements
		var $target = $(event.target);
		var $parent = $target.parent();
		var id      = $parent.data('id');

		this.println('Clicked agent "' + id + '"', 'ui');
		this.agentClickListener(id);
	},
	'setAgentClickListener': function(listener) {
		this.agentClickListener = listener;
	},
	'clearLog': function() {
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
};


// Constructor of Server class
function Server(host, port, id) {
	this.id  = id;
	this.url = host + ':' + port;

	this.connected = false;
	this.websocket = null;

	this.ui = new UIPanel();
	this.ui.setAgentClickListener((this.requestAgent).bind(this));
}

// Server methods
Server.prototype = {
	'onerror': function(event) {
		this.ui.println('WebSocket error', this.id);
		console.log('onerror (event next)');
		console.log(event);
	},
	'onmessage': function(event) {
		this.ui.println('Received message', this.id);
		var data = JSON.parse(event.data);

		// Switch what type of message is received
		// Act accordingly
		switch (data['command-id']) {
			case 'overview':
				var agent = data['content'];

				this.ui.addAgent(agent['id'], agent['type'], agent['state']);

				break;
			/*case 'EquipletAgent':
				break;*/
			default:
				this.ui.println('Message received with unknown type', this.id);
				break;
		}
	},
	'onopen': function(event) {
		this.ui.println('Connected to web socket', this.id);

		// Clear potential timeout trying to reconnect
		clearTimeout(this.timeout);

		this.ui.setConStatus(this.ui.OPEN);
		this.connected = true;

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

		console.log('onclose (event next)');
		console.log(event);

		this.ui.println('Connection closed with "' + desc + '"', this.id);

		this.ui.setConStatus(this.ui.CLOSED);
		this.connected = false;

		this.timeout = setTimeout((this.connect).bind(this), 3000);
	},

	'connect': function() {
		this.ui.println('Attempting to connect to "' + this.url + '"', this.id);
		this.ui.setConStatus(this.ui.CONNECTING);

		this.websocket = new WebSocket('ws://' + this.url);

		// Callbacks with correct context (this)
		this.websocket.onerror   = (this.onerror).bind(this);
		this.websocket.onmessage = (this.onmessage).bind(this);
		this.websocket.onclose   = (this.onclose).bind(this);
		this.websocket.onopen    = (this.onopen).bind(this);

		return this.websocket;
	},

	// Send to WS
	'send': function(command, id, values) {
		if (typeof(id)     === 'undefined') { id    = null; }
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
		if (!this.connected) {
			return;
		}

		this.ui.println('Get Agent "' + id + '" information from SCADA', this.id);

		this.send('GETINFO', id);
	},
};

// Server class allowing us to connect to web socket.
var server = new Server(window.location.hostname, 3529, 'SCADA');
server.connect();

/*window.addEventListener('beforeunload', function (e) {
	var msg = 'Leaving causes acquired data to be discarded';

	(e || window.event).returnValue = msg;
    return msg;
});*/
