"use strict";

function Grid(ui, $grid) {
    this.ui     = ui;
    this.$grid  = $grid;
    this.agents = {};
}

Grid.prototype = {
    'add': function(agent, click_listener) {
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
    },
    'update': function(agent) {
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
    },
    'remove': function(agent) {
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
    },


    'clean': function() {
        this.$grid.html('');
        this.agents = {};
    },
};

function Agent(ui, $agent) {
    this.ui     = ui;
    this.$agent = $agent;
    this.agent  = {};
    this.id     = null;
}

Agent.prototype = {
    'set': function(agent) {
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
    },
    'update': function(agent) {
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
    },


    'clean': function() {
        this.$agent.html('');
        this.agent  = {};
        this.id     = null;
    },
};


/**
 * @Author Benno Zeeman
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

UIPanel.prototype = {
    // Entities we can be connected with (internally)
    'ENTITY_GRID':  'grid',
    'ENTITY_AGENT': 'agent',

    // Connection statuses
    'CLOSED':      0,
    'CONNECTING':  1,
    'OPEN':        2,

    'setAgent': function(agent) {
        this.agent.set(agent);
    },
    'updateAgent': function(agent) {
        this.agent.update(agent);
    },

    'addToGrid': function(agent) {
        this.grid.add(agent, (this.clickAgent).bind(this));
    },
    'updateGrid': function(agent) {
        this.grid.update(agent);
    },
    'removeFromGrid': function(agent) {
        this.grid.remove(agent);
    },

    'clickNav': function(event) {
        event.preventDefault();

        var entity = $(event.target).data('entity');

        if (this.listener.UIEntity(entity) === true) {
            this.showEntity(entity);
        }
    },

    'showEntity': function(entity) {
        $('main > section').hide();
        $('#nav ul > li > a').removeClass('current');

        $('#nav ul > li > a[data-entity=' + entity + ']').addClass('current');
        $('main > section#' + entity).show();
    },

    'cleanEntities': function() {
        this.grid.clean();
        this.agent.clean();
    },


    'popup': function(msg) {
        var $div = $('<div></div>');
        $div.addClass('white-popup');
        $div.text(msg);

        $.magnificPopup.open({'items': {'src': $div, 'type': 'inline'}});
    },



    'println': function(msg, id) {
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

            this.btn_create_agent.prop('disabled', true);
        } else if (con_status == this.CONNECTING) {
            this.con_status_light.css('background-color', 'yellow');
            this.con_status_text.text('Connecting');
        } else if (con_status == this.OPEN) {
            this.con_status_light.css('background-color', 'green');
            this.con_status_text.text('Open');

            this.btn_create_agent.prop('disabled', false);
        }
    },

    'clickAgent': function(event) {
        event.preventDefault();

        // Retreive elements
        var id = $(event.currentTarget).parent('li').data('id');

        this.println('Clicked agent "' + id + '"', 'ui');

        this.listener.UIGetAgent(id);
    },
    'clickStop': function(event) {
        this.btn_stop.prop('disabled', true);
        this.btn_start.prop('disabled', false);

        this.listener.UIStop();
    },
    'clickStart': function(event) {
        this.btn_stop.prop('disabled', false);
        this.btn_start.prop('disabled', true);

        this.listener.UIStart();
    },

    'submitCreateAgent': function(event) {
        event.preventDefault();

        // Close form popup
        $.magnificPopup.close();

        // Agent object
        var agent = $('#create-agent-form').serializeObject();

        this.listener.UICreateAgent(agent);
    },
};
