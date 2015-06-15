"use strict";

/**
 * @Author Benno Zeeman
 */

function UIPanel() {
	this.el_log = $('#log');
}

UIPanel.prototype = {
	'writeLog': function(msg) {
		this.el_log.append(msg);
	},
};


function Server(url, id) {
	this.websocket = new WebSocket(url);
	this.form      = $('#' + id);

	this.panel = new UIPanel();

	//var that       = this;

	this.websocket.onerror   = (this.onerror).bind(this);
	this.websocket.onmessage = (this.onmessage).bind(this);
	this.websocket.onclose   = (this.onclose).bind(this);
	this.websocket.onopen    = (this.onopen).bind(this);
}

Server.prototype = {
	'onerror': function(event) {
		console.log('onerror');
		console.log(event);
	},
	'onmessage': function(event) {
		var data = JSON.parse(event.data);

		for (key in data) {
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
		}
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

		console.log(event);

		console.log(this.panel);
		console.log(this.form);

		this.panel.writeLog('onclose: "' + desc + '"');
	},
	'onopen': function(event) {
		this.websocket.send('get json');
	},
	'toJson': function() {

	},
};

var server = new Server('ws://' + window.location.hostname + ':3528', 'agent');
