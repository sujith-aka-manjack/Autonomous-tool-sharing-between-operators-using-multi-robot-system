#ifndef SCT_H
#define SCT_H

#include <stdlib.h>
#include <ctime>
#include <random>
#include <queue>
#include <unordered_map>
#include <functional>
#include <iostream>

/* Supervisor Info */
#define NUM_EVENTS 3
#define NUM_SUPERVISORS 1

/* Event Info */
#define EV_a 0

#define EV_c 1

#define EV_b 2


/* Variable Probability Info */
#define PROB_x 0

#define PROB_y 1

/* Structure to store member functions */
struct Scallback {
    std::function<void(void* data)> callback;
    std::function<unsigned char(void* data)> check_input;
    void* data;
};

/****************************************/
/*                 SCT                  */
/****************************************/

class SCT {

public:

    /* Class constructor */
    SCT();

    /* Class destructor */
    ~SCT();

    // void reset();

    /* Add callback function for a controllable event */
    template<typename Class>
    void add_callback(Class* p, unsigned char event, void (Class::*clbk)( void* ), void* empty_ci, void* data) {
        using namespace std::placeholders; //for _1, _2, _3...
        callback[event].callback    = std::bind(clbk, p, _1);
        callback[event].check_input = nullptr;
        callback[event].data        = data;
    }

    /* Add callback function for an uncontrollable event */
    template<typename Class>
    void add_callback(Class* p, unsigned char event, void* clbk, unsigned char (Class::*ci)( void* ), void* data) {
        using namespace std::placeholders; //for _1, _2, _3...
        callback[event].callback    = nullptr;
        callback[event].check_input = std::bind(ci, p, _1);
        callback[event].data        = data;
    }

    /* Add callback function for an uncontrollable event, while also triggering some robot function (e.g. turn LEDs on) */
    template<typename Class>
    void add_callback(Class* p, unsigned char event, void (Class::*clbk)( void* ), unsigned char (Class::*ci)( void* ), void* data) {            
        using namespace std::placeholders; //for _1, _2, _3...
        callback[event].callback    = std::bind(clbk, p, _1);
        callback[event].check_input = std::bind(ci, p, _1);
        callback[event].data        = data;
    }

    /* Run the generator player to execute the next action */
    virtual void run_step();

    virtual void print_current_state();

protected:

    /* Return whether an uncontrollable event has occured */
    virtual unsigned char input_read( unsigned char ev );

    /* Add uncontrollable events that have occured to the buffer */
    virtual void update_input();

    /* Given the supervisor and its state, return the position of the current state in the data structure */
    virtual unsigned long int get_state_position( unsigned char supervisor, unsigned long int state );

    /* Apply the transition from current state */
    virtual void make_transition( unsigned char event );

    /* Execute callback function */
    virtual void exec_callback( unsigned char ev );

    /* Return a uncontrollale event from the input buffer */
    virtual unsigned char get_next_uncontrollable( unsigned char *event );

    /* Choose a controllale event from the list of enabled controllable events */ 
    virtual unsigned char get_next_controllable( unsigned char *event );

    /* Return all the enabled controllable events */
    virtual unsigned char get_active_controllable_events( unsigned char *events );

    /* Map of callback functions */
    std::unordered_map<unsigned char, Scallback> callback;

    /* Buffer to record the occurances of uncontrollable events */
    std::queue<unsigned char> input_buffer;

    /* Supervisors */
    const unsigned char     ev_controllable[3] = { 1,0,1 };
    const unsigned char     sup_events[1][3] = { { 1,1,1 } };
    const unsigned long int sup_init_state[1]     = { 0 };
    unsigned long int       sup_current_state[1]  = { 0 };    
    const unsigned long int sup_data_pos[1] = { 0 };
    const unsigned char     sup_data[ 17 ] = { 2,EV_a,0,1,EV_c,0,0,3,EV_a,0,1,EV_c,0,1,EV_b,0,0 };

};

/****************************************/
/*               SCTProb                */
/****************************************/

class SCTProb : virtual public SCT {

public:

    /* Class constructor */
    SCTProb();

    /* Class destructor */
    ~SCTProb();

    /* Update variable probability */
    virtual void set_prob( unsigned char var_prob, float prob );

protected:

    /* Given the supervisor and its state, return the position of the current state's fixed probabilities in the data structure */
    virtual unsigned long int get_state_position_prob( unsigned char supervisor, unsigned long int state );

    /* Given the supervisor and its state, return the position of the current state's variable probabilities in the data structure */
    virtual unsigned long int get_state_position_var_prob( unsigned char supervisor, unsigned long int state );

    /* Choose a controllale event from the list of enabled controllable events using probabilities */
    virtual unsigned char get_next_controllable( unsigned char *event );

    /* Return all the enabled controllable event probabilities */
    virtual float get_active_controllable_events_prob( float *events );

    /* Probability info of supervisors */
    const unsigned long int sup_data_prob_pos[1] = { 0 };
    const float             sup_data_prob[ 5 ] = { 1,0.50000000,2,1.00000000,1.00000000 };
    const unsigned long int sup_data_var_prob_pos[1] = { 0 };
    const unsigned char     sup_data_var_prob[ 7 ] = { 1,PROB_y,2,PROB_x,PROB_y,1,PROB_x };
    float                   current_var_prob[2] = { 0.1,0.9 };
};

#endif