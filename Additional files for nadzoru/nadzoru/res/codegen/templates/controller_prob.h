/* Supervisor Info */
#define NUM_EVENTS {{ #events }}
#define NUM_SUPERVISORS {{ automata:len() }}

/* Event Info */
{% noblankline %}
{% for k_event, event in ipairs(events) %}
#define EV_{{event.name}} {{k_event-1}}
{% end %}
{% noblankline %}

/* Variable Probability Info */
#define PROB_x 0

/* Supervisors */
{% noblankline %}
{% with
    var_data          = {},
    var_data_pos      = {},
    var_state_map     = {},

    var_data_prob     = {},
    var_data_prob_pos = {},
    var_data_prob_pos_state = {},
    prob              = 0,
    num_controllable  = 0,

    var_data_var_prob = {},
    var_data_var_prob_pos = {},
%}
    {% for k_automaton, automaton in automata:ipairs() %}
        {% set var_data_pos[k_automaton]      = #var_data %}
        {% set var_data_prob_pos[k_automaton] = #var_data_prob %}
        {% set var_data_var_prob_pos[k_automaton] = #var_data_var_prob %}
        {% for k_state, state in automaton.states:ipairs() %}
            {% set var_state_map[ state ] = k_state - 1 %}
        {% end %}
        {% for k_state, state in automaton.states:ipairs() %}
            {% set var_data[#var_data +1] = state.transitions_out:len() %}
            {% set num_controllable       = 0 %}
            {% for k_transition, transition in state.transitions_out:ipairs() %}
                {% if transition.event.controllable %}
                    {% set num_controllable = num_controllable + 1 %}
                {% end %}
            {% end %}
            {% set var_data_prob[#var_data_prob +1] = num_controllable %}
            {% for k_transition, transition in state.transitions_out:ipairs() %}
                {% set var_data[#var_data +1] = 'EV_' .. transition.event.name %}
                {% set var_data[#var_data +1] = math.floor( var_state_map[ transition.target ] / 256 ) %}
                {% set var_data[#var_data +1] = var_state_map[ transition.target ] % 256 %}
                {% if transition.event.controllable %}
                    {% if transition.probability >= 1.0 %}
                        {% set var_data_prob[#var_data_prob +1] = "1" %}
                    {% else %}
                        {% set var_data_prob[#var_data_prob +1] = string.format( "%0.8f", transition.probability ) %}
                    {% end %}
                    {% set var_data_var_prob[#var_data_var_prob +1] = "0" %}
                {% end %}
            {% end %}

        {% end %}
    {% end %}
const unsigned char     ev_controllable[{{ #events }}] = { {% for k_event, event in ipairs(events) %}{{ event.controllable and 1 or 0 }}{% notlast %},{% end %} };
const unsigned char     sup_events[{{ automata:len() }}][{{ #events }}] = { {% for k_automaton, automaton in automata:ipairs() %}{ {% for i = 1, #events %}{{ sup_events[k_automaton][i] and 1 or 0 }}{% notlast %},{% end %} }{% notlast %},{% end %} };
const unsigned long int sup_init_state[{{ automata:len() }}]     = { {% for k_automaton, automaton in automata:ipairs() %}{{automaton.initial - 1}}{% notlast %},{% end %} };
unsigned long int       sup_current_state[{{ automata:len() }}]  = { {% for k_automaton, automaton in automata:ipairs() %}{{automaton.initial - 1}}{% notlast %},{% end %} };
const unsigned long int sup_data_pos[{{ automata:len() }}] = { {{ table.concat(var_data_pos, ',') }} };
const unsigned char     sup_data[ {{ #var_data }} ] = { {{ table.concat( var_data,',' ) }} };

/* Probability info of supervisors */
const unsigned long int sup_data_prob_pos[{{ automata:len() }}] = { {{ table.concat(var_data_prob_pos, ',') }} };
const float             sup_data_prob[ {{ #var_data_prob }} ] = { {{ table.concat( var_data_prob,',' ) }} };
const unsigned long int sup_data_var_prob_pos[{{ automata:len() }}] = { {{ table.concat(var_data_var_prob_pos, ',') }} };
const unsigned char     sup_data_var_prob[ {{ #var_data_var_prob }} ] = { {{ table.concat( var_data_var_prob,',' ) }} };
float                   current_var_prob[1] = { 1.0 };
{% endwith %}
{% endnoblankline %}
