"use strict";

/**
 * @Author Benno Zeeman
 */

/**
 * Server is a class that will use a web socket address to communicate
 * with a server to fetch and display Agents.
 * @param {string} host Host part of web socket address
 * @param {int}    port Port number of web socket address
 * @param {string} id   Identification string used in logging etc.
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


// Entities we can be connected with (internally)
Server.prototype.ENTITY_GRID  = 'grid';
Server.prototype.ENTITY_AGENT = 'agent';

// Commands
Server.prototype.CMD_GET_OVERVIEW      = 'GET_OVERVIEW';
Server.prototype.CMD_ADD_AGENT         = 'ADD_AGENT';
Server.prototype.CMD_GET_DETAILED_INFO = 'GET_DETAILED_INFO';
Server.prototype.CMD_UPDATE_AGENT      = 'UPDATE_AGENT';
Server.prototype.CMD_ON_TAKEDOWN       = 'ON_TAKEDOWN';

Server.prototype.CMD_GET_AGENT_INFO    = 'GET_AGENT_INFO';
Server.prototype.CMD_CREATE_AGENT      = 'CREATE_AGENT';


/**
 * onerror handle called by WebSocket
 */
Server.prototype.onerror = function(event) {
    if (!this.is_connected) {
        return;
    }
    this.ui.println('WebSocket error', this.id);
    //console.log('onerror (event next)');
    //console.log(event);
};
/**
 * onmessage handle called by WebSocket
 */
Server.prototype.onmessage = function(event) {
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
                //this.ui.disconnectAgent();
                this.setConnectedTo(this.ENTITY_GRID);
            }
            break;
        default:
            this.ui.println('Unknown "' + data['command'] + '"', this.id);
            break;
    }
};
/**
 * onopen handle called by WebSocket
 */
Server.prototype.onopen = function(event) {
    this.ui.println('Connected to web socket', this.id);

    // Clear potential timeout trying to reconnect
    clearTimeout(this.timeout);

    this.ui.setConStatus(this.ui.OPEN);
    this.is_connected = true;

    // Request grid overview from WS
    this.send(this.CMD_GET_OVERVIEW);
};
/**
 * onclose handle called by WebSocket
 */
Server.prototype.onclose = function(event) {
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
};



/**
 * Connect to the web socket address of this instance. Bind the WebSocket callbacks to the this instance.
 */
Server.prototype.connect = function() {
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
};
/**
 * Disconnect from the current WebSocket connection. Update UI and set status.
 */
Server.prototype.disconnect = function() {
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
};

/**
 * Send a specific command to the server, possibly with an agent object.
 * @param  {string} command One of the CMD_ strings that have certain meanings
 * @param  {string} agent   Agent object that is supplied with certain commands
 */
Server.prototype.send = function(command, agent) {
    // Build json object to be sent over to the server
    var json = {'command': command};
    if (typeof(agent) !== 'undefined' ) {
        json['agent'] = agent;
    }

    // Create JSON message to send over WS
    var str = JSON.stringify(json);

    this.websocket.send(str);
};

/**
 * Method that's called by the UIPanel when user wants to create an Agent
 */
Server.prototype.UICreateAgent = function(agent) {
    if (!this.is_connected) {
        return;
    }

    this.ui.println('Create Agent "' + agent['id'] + '"', this.id);

    this.send(this.CMD_CREATE_AGENT, agent);
};
/**
 * Method that's called by the UIPanel when user wants to get Agent information
 */
Server.prototype.UIGetAgent = function(id) {
    if (!this.is_connected) {
        return;
    }

    this.ui.println('Get Agent "' + id + '" information from SCADA', this.id);

    var agent = {'id': id};
    this.send(this.CMD_GET_AGENT_INFO, agent);
};

/**
 * Method that's called by the UIPanel when start button is pressed
 */
Server.prototype.UIStart = function() {
    this.connect();
};
/**
 * Method that's called by the UIPanel when stop button is pressed
 */
Server.prototype.UIStop = function() {
    this.disconnect();
};

/**
 * Method that's called by the UIPanel if users wants to switch to another entity (Grid vs Agent)
 */
Server.prototype.UIEntity = function(entity) {

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
};


/**
 * Set the Entity we're connected to (either Grid or Agent)
 */
Server.prototype.setConnectedTo = function(entity) {
    this.ui.cleanEntities();
    this.ui.showEntity(entity);

    this.connected_to = entity;
};
