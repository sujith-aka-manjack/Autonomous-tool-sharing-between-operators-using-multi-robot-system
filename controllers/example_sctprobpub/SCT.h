#ifndef SCT_H
#define SCT_H

#include <stdlib.h>
#include <ctime>
#include <random>
#include <queue>
#include <map>
#include <functional>
#include <iostream>

/* Supervisor Info */
#define NUM_EVENTS 4
#define NUM_SUPERVISORS 1

/* Event Info */
#define EV_a 0

#define EV__b 1

#define EV_c 2

#define EV_b 3

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
            std::cout << "Hi" << std::endl;

    }

    /* Run the generator player to execute the next action */
    virtual void run_step();

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
    std::map<unsigned char, Scallback> callback;

    /* Buffer to record the occurances of uncontrollable events */
    std::queue<unsigned char> input_buffer;

    /* Supervisors */
    const unsigned char     ev_controllable[4] = { 1,0,1,1 };
    const unsigned char     sup_events[1][4] = { { 1,1,1,1 } };
    const unsigned long int sup_init_state[1]     = { 0 };
    unsigned long int       sup_current_state[1]  = { 0 };
    const unsigned long int sup_data_pos[1] = { 0 };
    const unsigned char     sup_data[ 17 ] = { 3,EV_a,0,0,EV__b,0,0,EV_b,0,1,2,EV__b,0,0,EV_c,0,1 };
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

    /* Set the probability of a transition */
    virtual void set_event_prob( unsigned char supervisor, unsigned long int state, unsigned char event, float prob );

protected:

    /* Given the supervisor and its state, return the position of the current state's probabilities in the data structure */
    virtual unsigned long int get_state_position_prob( unsigned char supervisor, unsigned long int state );

    /* Choose a controllale event from the list of enabled controllable events using probabilities */
    virtual unsigned char get_next_controllable( unsigned char *event );

    /* Return all the enabled controllable event probabilities */
    virtual float get_active_controllable_events_prob( float *events );
    
    /* Probability info of supervisors */
    const unsigned long int sup_data_prob_pos[1] = { 0 };
    float sup_data_prob[ 5 ] = { 2,0.90000000,0.10000000,1,1 };

};

/****************************************/
/*                SCTPub                */
/****************************************/

class SCTPub : virtual public SCT {

public:

    /* Class constructor */
    SCTPub();

    /* Class destructor */
    ~SCTPub();

    /* Run the generator player to execute the next action */
    virtual void run_step();

protected:

    /* Add uncontrollable events that have occured to the buffer */
    virtual void update_input();

    /* Return a public uncontrollale event from the input buffer */
    virtual unsigned char get_next_uncontrollable_pub( unsigned char *event );

    /* Buffer to store public uncontrollable events */
    std::queue<unsigned char> input_buffer_pub;

    /* Public event info of supervisors */
    const unsigned char ev_shared[4] = { 0,1,0,1 };

};

/****************************************/
/*              SCTProbPub              */
/****************************************/

class SCTProbPub : public SCTProb, public SCTPub {

public:

    /* Class constructor */
    SCTProbPub();

    /* Class destructor */
    ~SCTProbPub();

protected:

};

#endif