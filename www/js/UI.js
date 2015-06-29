"use strict";

/**
 * @Author Benno Zeeman
 */

/**
 * Grid represents the Grid Entity (visualised in a tab on the page)
 * @param {UIPanel} UIPanel to call log method
 * @param {JQuery}  The JQuery element we display our information in
 */
function Grid(ui, $grid) {
    this.ui     = ui;
    this.$grid  = $grid;
    this.agents = {};
}

/**
 * Add an Agent to the Grid
 * @param {[object]}   agent          The Agent object to build the HTML from
 * @param {[callback]} click_listener Callback to bind to click event
 */
Grid.prototype.add = function(agent, click_listener) {
    var id = agent['id'];
    if (!id) {
        this.ui.println('No ID in Agent object', 'Grid.add');
        return;
    }

    this.ui.println('Adding "' + id + '"', 'Grid.add');

    // Create UI elements with attributes
    var $li = $('<li></li>', {'data-id': id, 'data-type': agent['type']});
    var $a  = $('<a></a>', {'href': '#', on: {click: click_listener}});

    $li.append($a);

    this.agents[id] = {$li: $li};

    for (var key in agent) {
        var $div = $('<div></div>');
        $div.attr('class', key);
        $div.text(agent[key]);

        $a.append($div);

        this.agents[id][key] = $div;
    }

    this.$grid.append($li);
};

/**
 * Update an Agent that's in the Grid
 * @param  {[type]} agent Agent object containing values to update Agent from
 */
Grid.prototype.update = function(agent) {
    var id = agent['id'];
    if (!id) {
        this.ui.println('No ID in Agent object', 'Grid.update');
        return;
    }

    if (id in this.agents) {

        // Loop through values in agent, so we can update them (usually just 1)
        for (var key in agent) {
            // ID is the ID that we used to find the agent, so updating it makes no sense.
            if (key === 'id') {
                continue;
            }


            // Check if value that we want to update is there
            if (key in this.agents[id]) {
                // Get the value of the property we want to update
                var value = agent[key];

                this.agents[id][key].text(value);
                this.ui.println('"' + id + '" updated "' + name + '"', 'Grid.update');
            } else {
                this.ui.println('"' + id + '" wanted to update inexistent "' + name + '"', 'Grid.update');
            }
        }
    } else {
        this.ui.println('Agent "' + id + '" not found', 'Grid.update');
    }
};
/**
 * Remove an Agent from the Grid
 * @param  {object} agent Agent object containing id used to find Agent to remove
 */
Grid.prototype.remove = function(agent) {
    var id = agent['id'];
    if (!id) {
        this.ui.println('No ID in Agent object', 'Grid.update');
        return;
    }

    if (id in this.agents) {
        this.agents[id].$li.remove();
    } else {
        this.ui.println('Agent "' + id + '" not found', 'Grid.remove');
    }
};

/**
 * Clean the Grid from all Agents
 */
Grid.prototype.clean = function() {
    this.$grid.html('');
    this.agents = {};
};




/**
 * Agent represents the Agent Entity (visualised in a tab on the page)
 * @param {UIPanel} UIPanel to call log method
 * @param {JQuery}  The JQuery element we display our information in
 */
function Agent(ui, $agent) {
    this.ui     = ui;
    this.$agent = $agent;
    this.agent  = {};
    this.id     = null;
}

/**
 * Set the Agent Entity from data
 * @param {object} agent Agent object to build the HTML with
 */
Agent.prototype.set = function(agent) {
    var id = agent['id'] ? agent['id']['value'] : null;
    if (!id) {
        this.ui.println('No ID in Agent object', 'Agent.set');
        return;
    }

    // Clear form
    this.$agent.text('');
    this.agent = {};
    this.id    = id;

    // Create 'div > input|select' for all keys in the agent object
    for (var name in agent) {
        if (!('type' in agent[name])) {
            //this.ui.println('No type given', 'Agent.set');
            continue;
        }

        var $div     = $('<div></div>');
        var $label   = $('<label></label>');
        var $element = null;

        switch (agent[name]['type']) {
            case 'string':
                $label.text(name);

                $element = $('<input />')
                    .attr('name', name)
                    .val(agent[name]['value'])
                    .attr('type', 'text')
                    .attr('disabled', agent[name]['read-only'])
                    .attr('required', agent[name]['required'])
                ;
                break;
            default:
                this.ui.println('Unknown type "' + agent[name]['type'] + '"', 'Agent.set');
                break;
        }

        if ($element !== null) {
            $div.append($label);
            $div.append($element);

            this.$agent.append($div);

            this.agent[name] = $element;
        }
    }

    var $div    = $('<div></div>');
    var $button = $('<button></button>')
        .attr('type', 'submit')
        .text('Submit')
    ;

    $div.append($button);

    this.$agent.append($div);
    this.ui.println('Form created for "' + id + '"', 'Agent.set');
};
/**
 * Update the current Agent's information
 * @param  {object} agent Agent object containing data used to update
 */
Agent.prototype.update = function(agent) {
    var id = agent['id'];
    if (!id) {
        this.ui.println('No ID in Agent object', 'Agent.set');
        return;
    }

    // Verify that we're the correct agent
    if (this.id === id) {
        for (var key in agent) {
            if (key === 'id') {
                continue;
            }

            // Check if form element exists that we want to update.
            if (key in this.agent) {
                var value = agent[key];

                this.agent[key].val(value);
            } else {
                this.ui.println('Could not update value "' + key + '" Agent "' + id + '"', 'Agent.update');
            }
        }
    } else {
        this.ui.println('Agent "' + id + '" not found', 'Agent.update');
    }
};

/**
 * Clean the Agent Entity
 */
Agent.prototype.clean = function() {
    this.$agent.html('');
    this.agent  = {};
    this.id     = null;
};


/**
 * UIPanel is the class used to update the interface the user is looking at.
 * It functions as a mediator between the Server instance and the user.
 * @param {Class} listener Listener implementing some UI callback methods
 */
function UIPanel(listener) {
    // Listener for certain events
    this.listener = listener;


    // Entities (sections) that are represented by classes
    this.grid  = new Grid(this, $('#grid > ul'));
    this.agent = new Agent(this, $('#agent > form'));


    // Navigation
    $('#nav ul > li > a').click((this.clickNav).bind(this));


    // Log section
    this.log_pre   = $('#log pre');
    this.log_clean = $('#log #clear');
    this.log_clean.click((this.clearLog).bind(this));


    // MagnificPopup opens form on pressing button
    this.btn_create_agent  = $('#grid #create-agent-btn');
    this.form_create_agent = $('#grid #create-agent-form');

    this.form_create_agent.submit((this.submitCreateAgent).bind(this));
    this.btn_create_agent.magnificPopup({'items': { 'src':  '#create-agent-form', 'type': 'inline' }});


    // Controls section
    this.con_status_light = $('#controls #con-status .light');
    this.con_status_text  = $('#controls #con-status .text');

    this.btn_stop  = $('#controls #stop').click((this.clickStop).bind(this));
    this.btn_start = $('#controls #start').click((this.clickStart).bind(this));

    this.setConStatus(this.CLOSED);
}

// Entities we can be connected with (internally)
UIPanel.prototype.ENTITY_GRID  = 'grid';
UIPanel.prototype.ENTITY_AGENT = 'agent';

// Connection statuses
UIPanel.prototype.CLOSED      = 0;
UIPanel.prototype.CONNECTING  = 1;
UIPanel.prototype.OPEN        = 2;

/**
 * Set the information of the Agent entity
 * @param {object} agent Agent object from which entity is built
 */
UIPanel.prototype.setAgent = function(agent) {
    this.agent.set(agent);
};
/**
 * Update the information of the Agent entity
 * @param {object} agent Agent object from which entity is updated
 */
UIPanel.prototype.updateAgent = function(agent) {
    this.agent.update(agent);
};

/**
 * Add an Agent to the Grid entity
 * @param {object} agent Agent object to be added to Grid
 */
UIPanel.prototype.addToGrid = function(agent) {
    this.grid.add(agent, (this.clickAgent).bind(this));
};
/**
 * Update an Agent in the Grid entity
 * @param {object} agent Agent object to be updated in the Grid
 */
UIPanel.prototype.updateGrid = function(agent) {
    this.grid.update(agent);
};
/**
 * Remove an Agent from the Grid
 * @param {object} agent Agent object having id that is to be removed
 */
UIPanel.prototype.removeFromGrid = function(agent) {
    this.grid.remove(agent);
};

/**
 * This method is called when a navigation button is clicked.
 */
UIPanel.prototype.clickNav = function(event) {
    event.preventDefault();

    var entity = $(event.target).data('entity');

    if (this.listener.UIEntity(entity) === true) {
        this.showEntity(entity);
    }
};

/**
 * Method that will show the entity matching the string given.
 * @param  {event} event Event from click event
 */
UIPanel.prototype.showEntity = function(entity) {
    $('main > section').hide();
    $('#nav ul > li > a').removeClass('current');

    $('#nav ul > li > a[data-entity=' + entity + ']').addClass('current');
    $('main > section#' + entity).show();
};
/**
 * Clean the entities, getting rid of Agent and Grid.
 */
UIPanel.prototype.cleanEntities = function() {
    this.grid.clean();
    this.agent.clean();
};

/**
 * Method wrapping the Magnific Popup so a message can be popupped
 * @param  {string} msg Message to show
 */
UIPanel.prototype.popup = function(msg) {
    var $div = $('<div></div>');
    $div.addClass('white-popup');
    $div.text(msg);

    $.magnificPopup.open({'items': {'src': $div, 'type': 'inline'}});
};


/**
 * Log a message with an identifier in the log window
 * @param  {string} msg Message to be printed
 * @param  {string} id  Identifier shown in front of message
 */
UIPanel.prototype.println = function(msg, id) {
    // If argument(s) not defined, give default
    id = id || '';

    // Moment.js gives us the formatting!
    var time_str = moment().format('HH:mm:ss.SSS');

    // If scrolled to bottom, remember and scroll down after new content
    var bottom = this.log_pre.prop('scrollHeight') - this.log_pre.scrollTop() == this.log_pre.innerHeight();

    // Create container with elements
    var $container =
        $('<div></div>', {'class': 'line'})
        .append($('<em></em>',     {'class': 'time'}).text(time_str))
        .append($('<em></em>',     {'class': 'id'}).text(id))
        .append($('<span></span>', {'class': 'msg'}).text(msg))
    ;

    // Write the log message in certain format
    this.log_pre.append($container);

    // Scroll down for 'sticky' effect.
    if (bottom === true) {
        this.log_pre.scrollTop(this.log_pre.prop('scrollHeight'));
    }
};
/**
 * Clear the log window.
 * @param  {event} event Event from click event from clear button
 */
UIPanel.prototype.clearLog = function(event) {
    if (typeof(event) !== 'undefined') {
        event.preventDefault();
    }

    this.log_pre.empty();
};

/**
 * Set the connection status in the control panel
 * @param {string} con_status Status, which is one of CLOSED, CONNECTING or OPEN
 */
UIPanel.prototype.setConStatus = function(con_status) {
    if (con_status == this.CLOSED) {
        this.con_status_light.css('background-color', 'red');
        this.con_status_text.text('Closed');

        this.btn_create_agent.prop('disabled', true);
    } else if (con_status == this.CONNECTING) {
        this.con_status_light.css('background-color', 'yellow');
        this.con_status_text.text('Connecting');
    } else if (con_status == this.OPEN) {
        this.con_status_light.css('background-color', 'green');
        this.con_status_text.text('Open');

        this.btn_create_agent.prop('disabled', false);
    }
};

/**
 * Method called wen an Agent is clicked in the Grid entity.
 * @param  {event} event Click event object
 */
UIPanel.prototype.clickAgent = function(event) {
    event.preventDefault();

    // Retreive elements
    var id = $(event.currentTarget).parent('li').data('id');

    this.println('Clicked agent "' + id + '"', 'ui');

    this.listener.UIGetAgent(id);
};

/**
 * Method called wen stop button is clicked in the control panel.
 * @param  {event} event Click event object
 */
UIPanel.prototype.clickStop = function(event) {
    this.btn_stop.prop('disabled', true);
    this.btn_start.prop('disabled', false);

    this.listener.UIStop();
};

/**
 * Method called wen start button is clicked in the control panel.
 * @param  {event} event Click event object
 */
UIPanel.prototype.clickStart = function(event) {
    this.btn_stop.prop('disabled', false);
    this.btn_start.prop('disabled', true);

    this.listener.UIStart();
};

/**
 * Method called wen submit button is clicked in the create agent form.
 * @param  {event} event Submit event object
 */
UIPanel.prototype.submitCreateAgent = function(event) {
    event.preventDefault();

    // Close form popup
    $.magnificPopup.close();

    // Agent object
    var agent = $('#create-agent-form').serializeObject();

    this.listener.UICreateAgent(agent);
};
