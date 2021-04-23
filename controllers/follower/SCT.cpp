#include "SCT.h"

/****************************************/
/*                 SCT                  */
/****************************************/

SCT::SCT(){
    std::srand(std::time(nullptr));
}

SCT::~SCT(){}

void SCT::run_step(){
    update_input(); // Get all uncontrollable events
    unsigned char event;

    /* Apply all the uncontrollable events */
    while ( get_next_uncontrollable( &event ) ){
        make_transition( event );
        exec_callback( event );
    }

    /* Apply the chosen controllable event */
    if( get_next_controllable( &event ) ){  /* Find and pick a controllable event (CE) */
        make_transition( event );
        exec_callback( event );
    }
}

void SCT::print_current_state() {
    std::cout << "sup [";
    for(size_t i = 0; i < NUM_SUPERVISORS; i++) {
        std::cout << sup_current_state[i];
        if(i < NUM_SUPERVISORS - 1)
            std::cout << ",";
    }
    std::cout << "]";
}

unsigned char SCT::input_read( unsigned char ev ){
    if( ev < NUM_EVENTS && callback[ ev ].check_input != NULL )
        return callback[ ev ].check_input( callback[ ev ].data );
    return 0;
}

void SCT::update_input(){
    unsigned char i;
    for(i=0;i<NUM_EVENTS;i++){
        if( !ev_controllable[i] ){   /* Check the UCEs only */
            if( input_read( i ) ){
                input_buffer.push(i);
            }
        }
    }
}

unsigned long int SCT::get_state_position( unsigned char supervisor, unsigned long int state ){
    unsigned long int position;
    unsigned long int s;
    unsigned long int en;
    position = sup_data_pos[ supervisor ];  /* Jump to the start position of the supervisor */
    for(s=0; s<state; s++){                 /* Keep iterating until the state is reached */
        en       = sup_data[position];      /* The number of transitions in the state */
        position += en * 3 + 1;             /* Next state position (Number transitions * 3 + 1) */
    }
    return position;
}

void SCT::make_transition( unsigned char event ){
    unsigned char i;
    unsigned long int position;
    unsigned char num_transitions;

    /* Apply transition to each local supervisor */
    for(i=0; i<NUM_SUPERVISORS; i++){
        if(sup_events[i][event]){   /* Check if the given event is part of this supervisor */
            
            /* Current state info of supervisor */
            position        = get_state_position(i, sup_current_state[i]);
            num_transitions = sup_data[position];
            position++; /* Point to first transition */

            /* Find the transition for the given event */
            while(num_transitions--){
                if(sup_data[position] == event){
                    sup_current_state[i] = (sup_data[position + 1] * 256) + (sup_data[position + 2]);
                    break;
                }
                position+=3;
            }
        }
    }
}

void SCT::exec_callback( unsigned char ev ){
    if( ev < NUM_EVENTS && callback[ ev ].callback != NULL )
        callback[ ev ].callback( callback[ ev ].data );
}

unsigned char SCT::get_next_uncontrollable( unsigned char *event ){
    if( !input_buffer.empty() ) {
        *event = input_buffer.front();
        input_buffer.pop();
        return 1;
    }
    return 0;
}

unsigned char SCT::get_next_controllable( unsigned char *event ){
    unsigned char events[NUM_EVENTS], i, count_actives;
    unsigned long int random_pos;
    
    /* Get controllable events that are enabled -> events */
    count_actives = get_active_controllable_events( events );

    if( count_actives ){                        /* If at least one event is enabled do */
        random_pos = rand() % count_actives;    /* Pick a random index (event) */
        for(i=0; i<NUM_EVENTS; i++){
            if( !random_pos && events[i] ){
                *event = i;
                return 1;
            } else if( events[i] ){
                random_pos--;
            }
        }
    }
    return 0;
}

unsigned char SCT::get_active_controllable_events( unsigned char *events ){
    unsigned char i,j;
    unsigned char count_actives = 0;

    /* Disable all non controllable events */
    for( i=0; i<NUM_EVENTS; i++ ){
        if( !ev_controllable[i] ){
            events[i] = 0;
        } else {
            events[i] = 1;
            count_actives++;
        }
    }

    /* Check if a controllable event is disabled in any of the supervisors */
    for(i=0; i<NUM_SUPERVISORS; i++){
        unsigned long int position;
        unsigned char ev_disable[NUM_EVENTS], k;
        unsigned char num_transitions;

        /* Init an array where all events are disabled */
        for(k=0; k < NUM_EVENTS;k++){
            ev_disable[k] = 1;  
        }

        /* Enable all events that are not part of this supervisor */
        for( j=0; j < NUM_EVENTS; j++ ){
            if( !sup_events[i][j] ){
                ev_disable[j] = 0;
            }
        }

        /* Get current state */
        position = get_state_position(i, sup_current_state[i]);
        num_transitions = sup_data[position];
        position++;

        /* Enable all events that have a transition from the current state */
        while(num_transitions--){
            ev_disable[ sup_data[position] ] = 0;
            position += 3;
        }

        /* Remove the controllable events to disable, leaving an array of enabled controllable events */
        for( j=0; j<NUM_EVENTS; j++ ){
            if( ev_disable[j] == 1 && events[ j ] ){
                events[ j ] = 0;
                count_actives--;
            }
        }
    }
    
    return count_actives;
}

/****************************************/
/*               SCTProb                */
/****************************************/

SCTProb::SCTProb(){}

SCTProb::~SCTProb(){}

void SCTProb::set_prob( unsigned char var_prob, float prob ) {
    current_var_prob[var_prob] = prob;
}

unsigned long int SCTProb::get_state_position_prob( unsigned char supervisor, unsigned long int state ) {
    unsigned long int s, en;
    unsigned long int prob_position = sup_data_prob_pos[ supervisor ];  /* Jump to the start position of the supervisor */
    for(s=0; s<state; s++){                                             /* Keep iterating until the state is reached */
        en            =  sup_data_prob[prob_position];                  /* The number of controllable events in the state */
        prob_position += en + 1;                                        /* Next controllable event's probability position */
    }
    return prob_position;
}

unsigned long int SCTProb::get_state_position_var_prob( unsigned char supervisor, unsigned long int state ) {
    unsigned long int s, t, en, pn;
    unsigned long int prob_position = sup_data_prob_pos[ supervisor ];
    unsigned long int var_prob_position = sup_data_var_prob_pos[ supervisor ];  /* Jump to the start position of the supervisor */
    for(s=0; s<state; s++){                                                     /* Keep iterating until the state is reached */
        en = sup_data_prob[prob_position];                                      /* The number of controllable events in the state */
        if(en == 0) {                                                           /* If there are no controllabel events, go to the next state */
            var_prob_position++;
            continue;
        }                                                        
        for(t=0; t<en; t++){                                                    
            pn = sup_data_var_prob[var_prob_position];                          /* The number of variable probabilities in the state */
            var_prob_position += pn + 1;                                        /* Next event's variable probability position */
        }
    }
    return var_prob_position;
}

unsigned char SCTProb::get_next_controllable( unsigned char *event ) {
    float events[NUM_EVENTS], random_value, random_sum = 0;
    unsigned long i;
    float prob_sum = get_active_controllable_events_prob( events );
    if( prob_sum > 0.0001 ){                /* If at least one event is enabled do */
        random_value = (float) rand() / RAND_MAX * prob_sum;   /* Pick a random index (event) */

        // std::cout << "prob_sum " << prob_sum << std::endl;
        // std::cout << "rand_val " << random_value << std::endl;
        for(i=0; i<NUM_EVENTS; i++)
            // std::cout << "events[" << i << "] " << events[i] << std::endl;

        for(i=0; i<NUM_EVENTS; i++){
            random_sum += events[i];        /* Add probability of each event until the random value is reached */
            if( (random_value < random_sum) && ev_controllable[ i ] ){
                *event = i;
                return 1;
            }
        }
    }
    return 0;
}

float SCTProb::get_active_controllable_events_prob( float *events ) {
    unsigned char i,j;
    float prob_sum = 0;

    /* Disable all non controllable events */
    for( i=0; i<NUM_EVENTS; i++ ){
        if( ev_controllable[i] ){
            events[i] = 1.0;
        } else {
            events[i] = 0;
        }
    }

    /* Check if a controllable event is disabled in any of the supervisors */
    for( i=0; i<NUM_SUPERVISORS; i++ ){
        unsigned long int position, position_prob, position_var_prob;
        unsigned char num_transitions;
        unsigned char ev_disable[NUM_EVENTS];

        for( j=0; j<NUM_EVENTS; j++ ){
            if( sup_events[i][j] ){
                ev_disable[j] = 1;  /* Unless this event has a transition in the current state, this event will be disabled*/

            } else {
                ev_disable[j] = 0;  /* If supervisor don't have this event, it can't disable the event*/
            }
        }

        /* Get current state and transition probabilities */
        position          = get_state_position(i, sup_current_state[i]);
        position_prob     = get_state_position_prob(i, sup_current_state[i]);
        position_var_prob = get_state_position_var_prob(i, sup_current_state[i]);
        num_transitions = sup_data[position];
        position++;
        position_prob++;

        /* Enable all events that have a transition from the current state */
        while(num_transitions--){
            unsigned char event = sup_data[position];

            /* Check if the event is controllable and exists in this supervisor */
            if( ev_controllable[ event ] && sup_events[i][ event ] ){
                ev_disable[ event ] = 0;
                events[ event ] *= sup_data_prob[position_prob];    /* Cumulatively multiply the event's probability from all supervisors */
                
                // std::cout << "\nsup_data_prob[" << position_prob << "] " << sup_data_prob[position_prob] << std::endl;
                // std::cout << "events[" << std::to_string(event) << "] " << events[event] << std::endl;
                // std::cout << "sup_data_var_prob[" << position_var_prob << "] " << std::to_string(sup_data_var_prob[position_var_prob]) << std::endl;

                unsigned long int num_var_prob = sup_data_var_prob[position_var_prob];

                /* If variable prob exists, multiply them one at a time */
                for( j=0; j<num_var_prob; j++ ){
                    position_var_prob++;

                    // std::cout << "sup_data_var_prob[" << position_var_prob << "] " << std::to_string(sup_data_var_prob[position_var_prob]) << std::endl;

                    unsigned char var_prob = sup_data_var_prob[position_var_prob];

                    // std::cout << "var_prob " << std::to_string(var_prob) << std::endl;
                    // std::cout << "position_var_prob " << position_var_prob << std::endl;
                    // std::cout << "current_var_prob[" << std::to_string(var_prob) << "] " << current_var_prob[var_prob] << std::endl;

                    events[ event ] *= current_var_prob[var_prob];
                }

                // std::cout << "events[" << std::to_string(event) << "] " << events[event] << std::endl;

                position_prob++;
                position_var_prob++;
            }
            position += 3;
        }

        /* Remove the controllable events to disable, leaving a list of enabled controllable events */
        for( j=0; j<NUM_EVENTS; j++ ){
            if( ev_disable[j] == 1 ){
                events[ j ] = 0;
            }
        }
    }

    /* Sum the probabilities */
    for( j=0; j<NUM_EVENTS; j++ ){
        prob_sum += events[ j ];
    }

    return prob_sum;
}
