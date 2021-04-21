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
#define NUM_EVENTS 11
#define NUM_SUPERVISORS 2

/* Event Info */
#define EV_flock 0

#define EV_singleChain 1

#define EV_wait 2

#define EV_leaderFar 3

#define EV_joinLeader 4

#define EV_multiChain 5

#define EV_stop 6

#define EV_LCFar 7

#define EV_leaderNear 8

#define EV_joinChain 9

#define EV_LCNear 10

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
    const unsigned char     ev_controllable[11] = { 1,0,1,0,1,0,1,0,0,1,0 };
    const unsigned char     sup_events[2][11] = { { 1,1,1,1,1,1,1,1,1,1,1 },{ 0,1,1,1,1,1,0,1,1,1,1 } };
    const unsigned long int sup_init_state[2]     = { 0,0 };
    unsigned long int       sup_current_state[2]  = { 0,0 };    
    const unsigned long int sup_data_pos[2] = { 0,55 };
    const unsigned char     sup_data[ 120 ] = { 3,EV_flock,0,1,EV_LCFar,0,0,EV_LCNear,0,0,3,EV_LCFar,0,1,EV_joinChain,0,2,EV_LCNear,0,1,5,EV_singleChain,0,2,EV_leaderFar,0,2,EV_multiChain,0,2,EV_stop,0,3,EV_leaderNear,0,2,6,EV_singleChain,0,3,EV_wait,0,3,EV_leaderFar,0,3,EV_joinLeader,0,0,EV_multiChain,0,3,EV_leaderNear,0,3,2,EV_LCFar,0,1,EV_LCNear,0,0,3,EV_LCFar,0,1,EV_LCNear,0,0,EV_joinChain,0,2,4,EV_singleChain,0,2,EV_leaderNear,0,3,EV_multiChain,0,4,EV_leaderFar,0,2,5,EV_singleChain,0,3,EV_leaderNear,0,3,EV_multiChain,0,3,EV_joinLeader,0,0,EV_leaderFar,0,2,6,EV_singleChain,0,2,EV_wait,0,4,EV_leaderNear,0,3,EV_multiChain,0,4,EV_joinLeader,0,0,EV_leaderFar,0,4 };
    
};

/****************************************/
/*               SCTProb                */
/****************************************/

/* Structure to store variable probability update functions */
struct Pcallback {
    std::function<float(void* data)> check_input;
    unsigned char supervisor;
    unsigned char state;
    unsigned char event;
    void* data;
};

class SCTProb : virtual public SCT {

public:

    /* Class constructor */
    SCTProb();

    /* Class destructor */
    ~SCTProb();

    /* Add callback function for updating variable probabilities */
    template<typename Class>
    void add_variable_prob(Class* p, unsigned char sup, unsigned char state, unsigned char event, float (Class::*ci)( void* ), void* data) {
        using namespace std::placeholders; //for _1, _2, _3...
        std::string key = std::to_string(sup) + '-' + std::to_string(state) + '-' + std::to_string(event);
        variable_prob_callback[key].check_input = std::bind(ci, p, _1);
        variable_prob_callback[key].supervisor  = sup;
        variable_prob_callback[key].state       = state;
        variable_prob_callback[key].event       = event;
        variable_prob_callback[key].data        = data;
    }

    /* Run the generator player to execute the next action */
    virtual void run_step();

protected:

    /* Get new variable probabilities from the robot */
    virtual void update_prob();

    /* Given the supervisor and its state, return the position of the current state's probabilities in the data structure */
    virtual unsigned long int get_state_position_prob( unsigned char supervisor, unsigned long int state );

    /* Choose a controllale event from the list of enabled controllable events using probabilities */
    virtual unsigned char get_next_controllable( unsigned char *event );

    /* Return all the enabled controllable event probabilities */
    virtual float get_active_controllable_events_prob( float *events );
    
    /* Map of callback functions for updating variable probabilities */
    std::unordered_map<std::string, Pcallback> variable_prob_callback;

    /* Probability info of supervisors */
    const unsigned long int sup_data_prob_pos[2] = { 0,9 };
    float                   sup_data_prob[ 18 ] = { 1,1,1,1,1,1,2,0.50000000,0.50000000,0,1,1,0,1,1,2,0.90000000,0.10000000 };


};

#endif