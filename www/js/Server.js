"use strict";

/**
 * @Author Benno Zeeman
 */
function Server(host, port, id) {
    this.id  = id;
    this.url = host + ':' + port;


    // When true will attempt reconnecting when disconnect
    this.retry = true;
    // Connection status
    this.is_connected = false;
    // The WebSocket instance
    this.websocket = null;

    // The UI object that calls our listening methods, and lets us visualize our actions
    this.ui = new UIPanel(this);

    this.ui.println('Server instantiated', this.id);

    this.connected_to = null;
}

// Server methods
Server.prototype = {
    // Entities we can be connected with (internally)
    'ENTITY_GRID':  'grid',
    'ENTITY_AGENT': 'agent',

    // Commands
    'CMD_GET_OVERVIEW':      'GET_OVERVIEW',
    'CMD_ADD_AGENT':         'ADD_AGENT',
    'CMD_GET_DETAILED_INFO': 'GET_DETAILED_INFO',
    'CMD_UPDATE_AGENT':      'UPDATE_AGENT',
    'CMD_ON_TAKEDOWN':       'ON_TAKEDOWN',

    'CMD_GET_AGENT_INFO':    'GET_AGENT_INFO',
    'CMD_CREATE_AGENT':      'CREATE_AGENT',

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
            // All agents that need to be added to Grid
            case this.CMD_GET_OVERVIEW:
                this.setConnectedTo(this.ENTITY_GRID);

                for(var i = 0; i < data['agents'].length; i++) {
                    var agent = data['agents'][i];
                    this.ui.addToGrid(agent);
                }

                break;
            // Specific agent that needs to be added to Grid
            case this.CMD_ADD_AGENT:
                this.ui.addToGrid(data['agent']);
                break;
            // Information about specific agent
            case this.CMD_GET_DETAILED_INFO:
                this.setConnectedTo(this.ENTITY_AGENT);

                this.ui.setAgent(data['agent']);
                break;
            // Update information for specific agent
            case this.CMD_UPDATE_AGENT:
                if (this.connected_to === this.ENTITY_GRID) {
                    this.ui.updateGrid(data['agent']);
                } else {
                    this.ui.updateAgent(data['agent']);
                }
                break;
            // Remove a specific agent
            case this.CMD_ON_TAKEDOWN:
                if (this.connected_to === this.ENTITY_GRID) {
                    this.ui.removeFromGrid(data['agent']);
                } else {
                    this.ui.disconnectAgent();
                }
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
        this.send(this.CMD_GET_OVERVIEW);
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

        if (this.retry) {
            this.timeout = setTimeout((this.connect).bind(this), 3000);
        }
    },



    'connect': function() {
        this.ui.println('Attempting to connect to "' + this.url + '"', this.id);
        this.ui.setConStatus(this.ui.CONNECTING);

        this.retry = true;

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

        this.ui.cleanGrid();

        this.is_connected      = false;
        this.retry = false;
        if (this.timeout !== null) {
            clearTimeout(this.timeout);
        }

        //if (this.websocket !== null) {
            this.websocket.close();
        //  this.websocket = null;
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


    'UICreateAgent': function(agent) {
        if (!this.is_connected) {
            return;
        }

        this.ui.println('Create Agent "' + agent['id'] + '"', this.id);

        this.send(this.CMD_CREATE_AGENT, agent);
    },
    'UIGetAgent': function(id) {
        if (!this.is_connected) {
            return;
        }

        this.ui.println('Get Agent "' + id + '" information from SCADA', this.id);

        var agent = {'id': id};
        this.send(this.CMD_GET_AGENT_INFO, agent);
    },

    'UIStart': function() {
        this.connect();
    },
    'UIStop': function() {
        this.disconnect();
    },

    'UIEntity': function(entity) {

        // If switchting entities is not 'allowed'
        if (!this.is_connected) {
            this.ui.popup('Not connected');
            return false;
        } else if (entity === this.ENTITY_AGENT) {
            this.ui.popup('Select an Agent from Grid');
            return false;
        }

        this.ui.cleanEntities();

        if (entity === this.ENTITY_GRID) {
            // Request grid overview from WS
            this.send(this.CMD_GET_OVERVIEW);
        }

        return true;
    },


    'setConnectedTo': function(entity) {
        this.ui.cleanEntities();
        this.ui.showEntity(entity);

        this.connected_to = entity;
    },

    /*'switch': function(agent) {
        // Re-request overview, and clean grid when leaving section
        if (agent === 'grid') {
            this.send(this.CMD_GET_OVERVIEW);
            this.ui.cleanAgent();
        } else if (agent === 'agent') {
            this.ui.cleanGrid();
        }
    },*/
};
