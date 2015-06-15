"use strict";

/**
 * @Author Benno Zeeman
 */

function UIPanel() {
	this.el_log = $('#log pre');
	this.grid   = $('#grid ul');

	this.agentClickListener = null;
}

UIPanel.prototype = {
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
		var bottom = this.el_log.prop('scrollHeight') - this.el_log.scrollTop() == this.el_log.innerHeight();


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
		this.el_log.append($container);

		// Scroll down for 'sticky' effect.
		if (bottom === true) {
			this.el_log.scrollTop(this.el_log.prop('scrollHeight'));
		}
	},
	'addAgent': function(aid, type, name) {
		this.println('Adding agent "' + aid + '"', 'ui');

		// Create UI elements with attributes
		var $li = $('<li></li>', {'data': {'aid': aid}, 'class': type});
		var $a  = $('<a></a>',   {'href': '#', on: {click: (this.agentClick).bind(this)}});

		// Set structure
		$li.append($a);
		$a.text(name);

		// Insert into UI
		this.grid.append($li);
	},
	'agentClick': function(event) {
		event.preventDefault();

		// Retreive elements
		var $target = $(event.target);
		var $parent = $target.parent();
		var aid     = $parent.data('aid');

		this.println('Clicked agent "' + aid + '"', 'ui');
		this.agentClickListener(aid);
	},
	'setAgentClickListener': function(listener) {
		this.agentClickListener = listener;
	}
};


// Constructor of Server class
function Server(url, id) {
	this.id        = id;
	this.websocket = new WebSocket(url);

	this.ui     = new UIPanel();
	this.ui.println('Server instantiated', this.id);
	this.ui.setAgentClickListener((this.requestAgent).bind(this));


	// Callbacks with correct context (this)
	this.websocket.onerror   = (this.onerror).bind(this);
	this.websocket.onmessage = (this.onmessage).bind(this);
	this.websocket.onclose   = (this.onclose).bind(this);
	this.websocket.onopen    = (this.onopen).bind(this);
}

// Server methods
Server.prototype = {
	'onerror': function(event) {
		this.ui.println('Error occurred', this.id);
		console.log('onerror (event next)');
		console.log(event);
	},
	'onmessage': function(event) {
		this.ui.println('Received message', this.id);
		var data = JSON.parse(event.data);

		// Switch what type of message is received
		// Act accordingly
		switch (data['type']) {
			case 'GridAgent':
				for(var i = 0; i < data['agents'].length; i++) {
					var agent = data['agents'][i];

					this.ui.addAgent(agent.ID, agent.type, agent.name);
				}

				break;
			case 'EquipletAgent':

				break;
			default:
				this.ui.println('Message received with unknown type', this.id);
				break;
		}


		/*for (key in data) {
			// Type to be filled according to type of field in JSON
			type = '';

			switch (data[key]['type']) {
				case 'int':
					//this.form.append($('<input type="integer" value="22132">'));
					type = 'integer';
				break;
				case 'string':
					//this.form.append($('<input type="">'));
					type = 'text';
				break;
				default:
					// Unknown type to us
					continue;
				break;
			}

			// Create label
			var label = $('<label></label>');
			label.text(key);

			// Create input field
			var input = $('<input>');

			input.attr('name', key);
			input.attr('value', data[key]['value']);
			input.attr('type', type);

			// Check if read only
			if (data[key]['read-only'] === true) {
				input.attr('readonly', 'readonly');
			}

			// Create the field and add it to form
			var field = $('<div></div>');
			field.append(label);
			field.append(input);

			this.form.append(field);
		}*/
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

		this.ui.println('onclose: "' + desc + '"' + ' (' + event.code + ')', this.id);
	},
	'onopen': function(event) {
		this.ui.println('Connected to web socket', this.id);

		// Request grid overview from WS
		this.send('GETOVERVIEW');

		// Adding test agents
		this.ui.addAgent('henk',   'equiplet', 'Henk');
		this.ui.addAgent('klaas',  'equiplet', 'Klaas');
		this.ui.addAgent('piet',   'product', 'Piet');
		this.ui.addAgent('jan',    'equiplet', 'Jan');
		this.ui.addAgent('gerrit', 'product', 'Gerrit');
		this.ui.addAgent('hans',   'equiplet', 'Hans');
	},
	// Send to WS
	'send': function(command, aid, values) {
		if (typeof(aid)    === 'undefined') { aid    = null; }
		if (typeof(values) === 'undefined') { values = []; }

		// Create JSON message to send over WS
		var str = JSON.stringify({
			'command': command,
			'aid':     aid,
			'values':  values,
		});
		this.websocket.send(str);
	},
	'toJson': function() {

	},
	'requestAgent': function(aid) {
		this.ui.println('Get Agent "' + aid + '" information from SCADA', this.id);

		this.send('GETINFO', aid);
	},
};

// Instantiate Server to connect with SCADA system
var server = new Server('ws://' + window.location.hostname + ':3529', 'SCADA');
