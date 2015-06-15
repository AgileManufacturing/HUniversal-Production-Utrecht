"use strict";

/**
 * @Author Benno Zeeman
 */

function UIPanel() {
	this.el_log = $('#log pre');
	this.grid   = $('#grid ul');


}

UIPanel.prototype = {
	'println': function(msg, id) {
		var id_str = '';
		if (typeof id !== 'undefined') {
			id_str = '<em class="id">' + id + '</em> ';
		}

		var date = new Date();

		var hh = date.getHours();
		var mm = date.getMinutes();
		var ss = date.getSeconds();
		var cc = Math.floor(date.getMilliseconds() / 10);

		if (hh < 10) { hh = '0' + hh; }
		if (mm < 10) { mm = '0' + mm; }
		if (ss < 10) { ss = '0' + ss; }
		if (cc < 10) { cc = '0' + cc; }

		var str = hh + ':' + mm + ':' + ss + '.' + cc;



		var bottom = this.el_log.prop('scrollHeight') - this.el_log.scrollTop() == this.el_log.innerHeight();

		this.el_log.append(
			'<em class="time">' + str + '</em> '
			+ id_str
			+ msg
			+ '\n'
		);

		if (bottom === true) {
			this.el_log.scrollTop(this.el_log.prop('scrollHeight'));
		}
	},
	'addAgent': function(id, type, name) {
		this.println('Adding agent "' + id + '"', 'ui');

		var $li = $('<li></li>', {'data-id': id, 'class': type});
		var $a  = $('<a></a>',   {'href': '#', on: {click: (this.agentClick).bind(this)}});

		$li.append($a);
		$a.text(name);

		this.grid.append($li);
	},
	'agentClick': function(event) {
		event.preventDefault();

		var $target = $(event.target);
		var id      = $target.data('id');
		this.println('Clicked agent "' + id + '"', 'ui');



	},
};


function Server(url, id) {
	this.id        = id;
	this.websocket = new WebSocket(url);

	this.panel     = new UIPanel();
	this.panel.println('Server instantiated', this.id);

	this.websocket.onerror   = (this.onerror).bind(this);
	this.websocket.onmessage = (this.onmessage).bind(this);
	this.websocket.onclose   = (this.onclose).bind(this);
	this.websocket.onopen    = (this.onopen).bind(this);
}

Server.prototype = {
	'onerror': function(event) {
		this.panel.println('Error occurred', this.id);
		console.log('onerror (event next)');
		console.log(event);
	},
	'onmessage': function(event) {
		this.panel.println('Received message', this.id);
		var data = JSON.parse(event.data);

		switch (data['type']) {
			case 'grid':
				for(var i = 0; i < data['agents'].length; i++) {
					var agent = data['agents'][i];

					this.panel.addAgent(agent.ID, agent.type, agent.name);
				}

				break;
			case 'equiplet':

				break;
			default:
				this.panel.println('Message received with unknown type', this.id);
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

		this.panel.println('onclose: "' + desc + '"' + ' (' + event.code + ')', this.id);
	},
	'onopen': function(event) {
		var str = JSON.stringify({
			'command': 'GETOVERVIEW',
			'aid':     null,
			'values':  []
		});
		this.websocket.send(str);
		this.panel.println('Connected to web socket', this.id);

		this.panel.addAgent('henk',   'equiplet', 'Henk');
		this.panel.addAgent('klaas',  'equiplet', 'Klaas');
		this.panel.addAgent('piet',   'equiplet', 'Piet');
		this.panel.addAgent('jan',    'equiplet', 'Jan');
		this.panel.addAgent('gerrit', 'equiplet', 'Gerrit');
		this.panel.addAgent('hans',   'equiplet', 'Hans');
	},
	'toJson': function() {

	},
};

var server = new Server('ws://' + window.location.hostname + ':3529', 'SCADA');
