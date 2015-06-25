"use strict";

$.fn.serializeObject = function()
{
    var o = {};
    var a = this.serializeArray();
    $.each(a, function() {
        if (o[this.name] !== undefined) {
            if (!o[this.name].push) {
                o[this.name] = [o[this.name]];
            }
            o[this.name].push(this.value || '');
        } else {
            o[this.name] = this.value || '';
        }
    });
    return o;
};

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
	this.grid           = $('#grid ul');
	$('#grid #create-agent-btn').magnificPopup({
		'items': {
			'src':  '#create-agent-form',
			'type': 'inline'
		}
	});
	$('#grid #create-agent-form').submit((this.submitCreateAgent).bind(this));
	this.create_agent_listener  = null;

	this.click_agent_listener   = null;


	// Agent section
	this.agent = {
		'id':       $('#agent .id'),
		'schedule': $('#agent .schedule'),
		'state':    $('#agent .state'),
		'type':     $('#agent .type'),
		'mode':     $('#agent .mode'),
	};

	// Controls section
	this.con_status_light = $('#controls #con-status .light');
	this.con_status_text  = $('#controls #con-status .text');

	this.btn_stop  = $('#controls #stop').click((this.clickStop).bind(this));
	this.btn_start = $('#controls #start').click((this.clickStart).bind(this));

	this.click_btn_stop_listener  = null;
	this.click_btn_start_listener = null;

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
		id = (typeof id === 'undefined') ? '' : id;

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
	'addAgent': function(agent) {
		// Make sure properties are defined
		agent['id'   ] = 'id'    in agent ? agent['id'   ] : null;
		agent['type' ] = 'type'  in agent ? agent['type' ] : null;
		agent['state'] = 'state' in agent ? agent['state'] : null;

		if (agent['id'] === null) {
			this.println('No ID to work with', 'ui');
			return;
		}

		var id   = agent['id'];
		var type = agent['type'];

		if (id in this.agents) {
			this.println('Updating agent "' + id + '"', 'ui');
		} else {
			this.println('Adding new agent "' + id + '"', 'ui');

			// Create UI elements with attributes
			var $li = $('<li></li>', {'data-id': id, 'data-type': type});
			var $a  = $('<a></a>', {'href': '#', on: {click: (this.clickAgent).bind(this)}});

			var $id    = $('<div></div>', {'class': 'id'});
			var $type  = $('<div></div>', {'class': 'type'});
			var $state = $('<div></div>', {'class': 'state'});

			$li.append($a);
			$a
				.append($id)
				.append($type)
				.append($state)
			;

			// Insert into UI
			this.grid.append($li);

			this.agents[id] = {
				'li':    $li,
				'a':     $a,
				'id':    $id,
				'type':  $type,
				'state': $state,
			};
		}

		// Obtuse syntax, but clean
		agent['id'   ] === null || this.agents[id]['id'   ].text(agent['id'   ]);
		agent['type' ] === null || this.agents[id]['type' ].text(agent['type' ]);
		agent['state'] === null || this.agents[id]['state'].text(agent['state']);
	},

	'setAgent': function(agent) {
		// Define properties when inexistent
		agent['id'      ] = ('id'       in agent) ? agent['id'      ] : null;
		agent['schedule'] = ('schedule' in agent) ? agent['schedule'] : null;
		agent['state'   ] = ('state'    in agent) ? agent['state'   ] : null;
		agent['type'    ] = ('type'     in agent) ? agent['type'    ] : null;
		agent['mode'    ] = ('mode'     in agent) ? agent['mode'    ] : null;

		// Obtuse syntax, but clean
		agent['id'      ] === null || this.agent['id'      ].text('').text(agent['id'      ]);
		agent['schedule'] === null || this.agent['schedule'].text('').text(agent['schedule']);
		agent['state'   ] === null || this.agent['state'   ].text('').text(agent['state'   ]);
		agent['type'    ] === null || this.agent['type'    ].text('').text(agent['type'    ]);
		agent['mode'    ] === null || this.agent['mode'    ].text('').text(agent['mode'    ]);
		// Switch to the agent tab after we set the new data to the Agent section
		this.switchSection('agent');
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
		var $a  = $(event.currentTarget);
		var $li = $a.parent('li');
		var id  = $li.data('id');

		this.println('Clicked agent "' + id + '"', 'ui');

		if (this.click_agent_listener !== null) {
			this.click_agent_listener(id);
		}
	},
	'clickStop': function(event) {
		this.btn_stop.prop('disabled', true);
		this.btn_start.prop('disabled', false);

		if (this.click_btn_stop_listener !== null) {
			this.click_btn_stop_listener();
		}
	},
	'clickStart': function(event) {
		this.btn_stop.prop('disabled', false);
		this.btn_start.prop('disabled', true);

		if (this.click_btn_start_listener !== null) {
			this.click_btn_start_listener();
		}
	},
	'setClickAgentListener':  function(listener) { this.click_agent_listener      = listener; },
	'setClickStopListener':   function(listener) { this.click_btn_stop_listener   = listener; },
	'setClickStartListener':  function(listener) { this.click_btn_start_listener  = listener; },
	'setCreateAgentListener': function(listener) { this.create_agent_listener     = listener; },

	'submitCreateAgent': function(event) {
		event.preventDefault();

		$.magnificPopup.close();

		if (this.create_agent_listener !== null) {
			this.create_agent_listener($('#create-agent-form').serializeObject());
		}
	},

	'navClick': function(event) {
		event.preventDefault();

		var $a = $(event.target);

		this.switchSection($a);
	},
	'switchSection': function(section) {
		$('main > section').hide();
		$('#nav ul > li > a').removeClass('current');

		var $a;

		if (section instanceof jQuery) {
			$a      = section;
			section = $a.data('section');
		} else {
			$a = $('#nav ul > li > a[data-section=' + section + ']');
		}

		$a.addClass('current');
		$('main > section#' + section).show();
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
	this.ui.setClickStopListener((this.disconnect).bind(this));
	this.ui.setClickStartListener((this.connect).bind(this));
	this.ui.setCreateAgentListener((this.createAgent).bind(this));

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

		this.ui.println('Received "' + data['command'] + '"', this.id);

		// Switch what type of message is received
		// Act accordingly
		switch (data['command']) {
			case 'GET_OVERVIEW':
			 	for(var i = 0; i < data['agents'].length; i++) {
 					var agent = data['agents'][i];
					this.ui.addAgent(agent);
 				}

				break;
			case 'UPDATEAGENT':
			case 'ADDAGENT':
				this.ui.addAgent(data['agent']);

				break;
			case 'GET_DETAILED_INFO':
				this.ui.setAgent(data['agent']);

				break;
			default:
				this.ui.println('Unknown "' + data['command'] + '"', this.id);
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
		this.send('GET_OVERVIEW');
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
	'send': function(command, agent) {
		var json = {'command': command};
		if (typeof(agent) !== 'undefined' ) {
			json['agent'] = agent;
		}

		// Create JSON message to send over WS
		var str = JSON.stringify(json);

		this.websocket.send(str);
	},

	'toJson': function() {

	},

	'createAgent': function(agent) {
		if (!this.is_connected) {
			return;
		}

		this.ui.println('Create Agent "' + agent['id'] + '"', this.id);

		this.send('CREATE_AGENT', agent);
	},
	'requestAgent': function(id) {
		if (!this.is_connected) {
			return;
		}

		this.ui.println('Get Agent "' + id + '" information from SCADA', this.id);

		var agent = {'id': id};
		this.send('GET_AGENT_INFO', agent);
	},
};

// Server class allowing us to connect to web socket.
var server = new Server(window.location.hostname, 3529, 'SCADA');
//server.connect();
