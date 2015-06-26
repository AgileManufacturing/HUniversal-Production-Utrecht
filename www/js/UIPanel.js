"use strict";

/**
 * @Author Benno Zeeman
 */
function UIPanel() {
    // Navigation
    $('#nav ul > li > a').click((this.navClick).bind(this));
    this.switch_listener = null;



    // Log section
    this.log_pre   = $('#log pre');
    this.log_clean = $('#log #clear');
    this.log_clean.click((this.clearLog).bind(this));



    // Grid section
    this.grid_list = $('#grid ul');
    this.cleanGrid();

    // MagnificPopup opens form on pressing button
    this.btn_create_agent  = $('#grid #create-agent-btn');
    this.form_create_agent = $('#grid #create-agent-form');

    this.form_create_agent.submit((this.submitCreateAgent).bind(this));
    this.btn_create_agent.magnificPopup({'items': { 'src':  '#create-agent-form', 'type': 'inline' }});

    this.create_agent_listener = null;

    this.click_agent_listener = null;



    // Agent section
    this.agent_form = $('#agent > form');
    this.cleanAgent();



    // Controls section
    this.con_status_light = $('#controls #con-status .light');
    this.con_status_text  = $('#controls #con-status .text');

    this.btn_stop  = $('#controls #stop').click((this.clickStop).bind(this));
    this.btn_start = $('#controls #start').click((this.clickStart).bind(this));

    this.click_stop_listener  = null;
    this.click_start_listener = null;

    this.setConStatus(this.CLOSED);
}

UIPanel.prototype = {
    'CLOSED':      0,
    'CONNECTING':  1,
    'OPEN':        2,
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


    'addAgent': function(agent) {
        var id = agent['id'];
        if (!id) {
            this.println('No ID to work with', 'ui');
            return;
        } else {
            this.println('Adding Agent "' + id + '" to Grid section', 'ui');
        }

        this.println('Adding "' + id + '"', 'ui');

        // Create UI elements with attributes
        var $li = $('<li></li>', {'data-id': id, 'data-type': agent['type']});
        var $a  = $('<a></a>', {'href': '#', on: {click: (this.clickAgent).bind(this)}});

        $li.append($a);

        this.agents[id] = {};

        for (var p in agent) {
            var $div = $('<div></div>');
            $div.attr('class', p);
            $div.text(agent[p]);

            $a.append($div);

            this.agents[id][p] = $div;
        }

        this.grid_list.append($li);
    },

    'updateAgent': function(agent) {
        var id = agent['id'];
        if (!id) {
            this.println('No ID to work with', 'ui');
            return;
        } else {
            this.println('Updating Agent "' + id + '"', 'ui');
        }

        // Check where agent is in UI
        if (this.agent['id'] === id) {
            for (var p in agent) {
                if (p === 'id') {
                    continue;
                }


                // CHeck if form element exists that we want to update.
                if (p in this.agent['fields']) {
                    var value = agent[p];

                    this.agent['fields'][p].val(value);
                } else {
                    this.println('Could not update value "' + p + '" Agent "' + id + '"', 'ui');
                }
            }
        } else if (id in this.agents) {
            this.println('Updating Agent in Grid', 'ui');

            // Loop through values in agent, so we can update them (usually just 1)
            for (var p in agent) {
                // ID is the ID that we used to find the agent, so updating it makes no sense.
                if (p === 'id') {
                    continue;
                }


                // Check if value that we want to update is there
                if (p in this.agents[id]) {
                    // Get the value of the property we want to update
                    var value = agent[p];

                    this.agents[id][p].text(value);
                } else {
                    this.println('Could not update value "' + p + '" Agent "' + id + '" in Grid section', 'ui');
                }
            }
        } else {
            this.println('Agent not found in Grid nor Agent section', 'ui');
        }
    },

    'setAgent': function(agent) {
        var id = agent['id'] ? agent['id']['value'] : null;
        if (!id) {
            this.println('No ID to work with', 'ui');
            return;
        } else {
            this.println('Setting Agent "' + id + '" to Agent section', 'ui');
        }

        // Clear form
        this.agent_form.text('');
        this.agent = {'id': id, 'fields': {}};

        // Create 'div > input|select' for all properties
        for (var name in agent) {
            var $div     = $('<div></div>');
            var $label   = $('<label></label>');
            var $element = null;

            if (!('type' in agent[name])) {
                this.println('No type given', 'ui');
                continue;
            }

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
                    this.println('Unknown type "' + agent[name]['type'] + '"', 'ui');
                    break;
            }

            if ($element !== null) {
                $div.append($label);
                $div.append($element);

                this.agent_form.append($div);

                this.agent['fields'][name] = $element;
            }
        }

        var $div    = $('<div></div>');
        var $button = $('<button></button>');
        $button.attr('type', 'submit');
        $button.text('Submit');

        $div.append($button);

        this.agent_form.append($div);
        this.switch('agent');
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

        if (this.click_stop_listener !== null) {
            this.click_stop_listener();
        }
    },
    'clickStart': function(event) {
        this.btn_stop.prop('disabled', false);
        this.btn_start.prop('disabled', true);

        if (this.click_start_listener !== null) {
            this.click_start_listener();
        }
    },

    'submitCreateAgent': function(event) {
        event.preventDefault();

        $.magnificPopup.close();

        console.log(this.create_agent_listener);

        if (this.create_agent_listener !== null) {
            this.create_agent_listener($('#create-agent-form').serializeObject());
        }
    },

    'navClick': function(event) {
        event.preventDefault();

        var section = $(event.target).data('section');

        this.switch(section);
    },
    'switch': function(section) {
        $('main > section').hide();
        $('#nav ul > li > a').removeClass('current');

        $('#nav ul > li > a[data-section=' + section + ']').addClass('current');
        $('main > section#' + section).show();

        if (this.switch_listener !== null) {
            this.switch_listener(section);
        }
    },

    'cleanGrid': function() {
        this.grid_list.html('');

        this.agents = {};
    },
    'cleanAgent': function() {
        this.agent_form.html();
        this.agent_form.removeAttr('id');

        this.agent = {'id': null, 'fields': {}};
    },
};
