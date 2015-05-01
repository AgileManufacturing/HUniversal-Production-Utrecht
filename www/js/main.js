function Server(url, id) {
	this.websocket = new WebSocket(url);
	this.form      = $('#' + id);

	var that = this;

	this.websocket.onerror   = function(event) { that.onerror(event); };
	this.websocket.onmessage = function(event) { that.onmessage(event); };
	this.websocket.onclose   = function(event) { that.onclose(event); };
	this.websocket.onopen    = function(event) { that.onopen(event); };
}

Server.prototype = {
	'onerror': function(event) {

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

	},
	'onopen': function(event) {
		this.websocket.send('get json');
	},
	'toJson': function() {

	},
};

var server = new Server('ws://' + window.location.hostname + ':5000', 'agent');
