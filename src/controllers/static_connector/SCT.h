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
#define NUM_EVENTS 12
#define NUM_SUPERVISORS 3

/* Event Info */
#define EV_moveStop 0

#define EV_setFS 1

#define EV_moveFlock 2

#define EV_setCS 3

#define EV_taskBegin 4

#define EV_taskStop 5

#define EV_receiveTB 6

#define EV_receiveTS 7

#define EV_distNear 8

#define EV_isNearest 9

#define EV_notNearest 10

#define EV_distFar 11

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
    const unsigned char     ev_controllable[12] = { 1,1,1,1,1,1,0,0,0,0,0,0 };
    const unsigned char     sup_events[3][12] = { { 1,1,1,1,0,0,0,0,0,0,0,0 },{ 0,1,0,1,1,1,1,1,0,0,0,0 },{ 0,1,0,1,0,0,0,0,1,1,1,1 } };
    const unsigned long int sup_init_state[3]     = { 0,0,0 };
    unsigned long int       sup_current_state[3]  = { 0,0,0 };    
    const unsigned long int sup_data_pos[3] = { 0,16,79 };
    const unsigned char     sup_data[ 189 ] = { 1,EV_moveFlock,0,1,1,EV_setCS,0,2,1,EV_moveStop,0,3,1,EV_setFS,0,0,3,EV_setCS,0,1,EV_receiveTB,0,2,EV_receiveTS,0,0,3,EV_setFS,0,0,EV_receiveTB,0,1,EV_receiveTS,0,1,3,EV_taskBegin,0,3,EV_receiveTB,0,2,EV_receiveTS,0,0,2,EV_receiveTB,0,3,EV_receiveTS,0,4,4,EV_setCS,0,5,EV_taskStop,0,0,EV_receiveTB,0,3,EV_receiveTS,0,4,4,EV_taskStop,0,1,EV_setFS,0,4,EV_receiveTB,0,5,EV_receiveTS,0,5,4,EV_distNear,0,0,EV_isNearest,0,1,EV_notNearest,0,0,EV_distFar,0,2,4,EV_distNear,0,1,EV_isNearest,0,1,EV_notNearest,0,0,EV_distFar,0,3,4,EV_distNear,0,0,EV_isNearest,0,3,EV_notNearest,0,2,EV_distFar,0,2,5,EV_distNear,0,1,EV_isNearest,0,3,EV_setCS,0,4,EV_notNearest,0,2,EV_distFar,0,3,4,EV_distNear,0,5,EV_isNearest,0,4,EV_notNearest,0,6,EV_distFar,0,4,4,EV_distNear,0,5,EV_isNearest,0,5,EV_notNearest,0,7,EV_distFar,0,4,4,EV_distNear,0,7,EV_isNearest,0,4,EV_notNearest,0,6,EV_distFar,0,6,5,EV_distNear,0,7,EV_isNearest,0,5,EV_setFS,0,0,EV_notNearest,0,7,EV_distFar,0,6 };

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
    const unsigned long int sup_data_prob_pos[3] = { 0,8,21 };
    const float             sup_data_prob[ 31 ] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,2,0.50000000,0.50000000,2,0.50000000,0.50000000,0,0,0,1,1,0,0,0,1,1 };
    const unsigned long int sup_data_var_prob_pos[3] = { 0,4,11 };
    const unsigned char     sup_data_var_prob[ 13 ] = { 0,0,0,0,0,0,0,0,0,0,0,0,0 };
    float                   current_var_prob[0] = {  };
};

#endif