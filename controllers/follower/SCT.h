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
#define NUM_EVENTS 15
#define NUM_SUPERVISORS 6

/* Event Info */
#define EV_moveStop 0

#define EV_moveFlock 1

#define EV_setF 2

#define EV_setC 3

#define EV_receiveR 4

#define EV_sendA 5

#define EV_sendR 6

#define EV_receiveNA 7

#define EV_receiveA 8

#define EV_notCondC1 9

#define EV_condC1 10

#define EV_notCondC2 11

#define EV_condC2 12

#define EV_notCondC3 13

#define EV_condC3 14


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
    const unsigned char     ev_controllable[15] = { 1,1,1,1,0,1,1,0,0,0,0,0,0,0,0 };
    const unsigned char     sup_events[6][15] = { { 1,1,1,1,0,0,0,0,0,0,0,0,0,0,0 },{ 1,1,1,1,1,1,1,1,1,0,0,0,0,0,0 },{ 0,0,0,0,0,1,1,0,0,1,1,0,0,0,0 },{ 0,0,0,0,0,1,1,0,0,0,0,1,1,0,0 },{ 0,0,0,0,0,1,1,0,0,0,0,0,0,1,1 },{ 0,0,1,1,1,1,1,1,1,0,0,0,0,0,0 } };
    const unsigned long int sup_init_state[6]     = { 0,0,0,0,0,0 };
    unsigned long int       sup_current_state[6]  = { 0,0,0,0,0,0 };    
    const unsigned long int sup_data_pos[6] = { 0,25,168,185,202,219 };
    const unsigned char     sup_data[ 267 ] = { 2,EV_moveFlock,0,1,EV_setC,0,2,2,EV_moveStop,0,0,EV_setC,0,3,1,EV_setF,0,0,2,EV_moveStop,0,2,EV_setF,0,1,5,EV_receiveR,0,1,EV_sendR,0,2,EV_receiveNA,0,0,EV_moveFlock,0,3,EV_receiveA,0,0,4,EV_receiveR,0,1,EV_receiveNA,0,1,EV_moveFlock,0,4,EV_receiveA,0,1,4,EV_receiveR,0,1,EV_receiveNA,0,2,EV_moveFlock,0,5,EV_receiveA,0,2,4,EV_receiveR,0,4,EV_sendR,0,5,EV_receiveNA,0,3,EV_receiveA,0,3,4,EV_receiveR,0,4,EV_receiveNA,0,4,EV_moveStop,0,6,EV_receiveA,0,4,4,EV_receiveR,0,4,EV_receiveNA,0,5,EV_moveStop,0,7,EV_receiveA,0,5,4,EV_receiveR,0,6,EV_sendA,0,8,EV_receiveNA,0,6,EV_receiveA,0,6,3,EV_receiveR,0,6,EV_receiveNA,0,9,EV_receiveA,0,8,4,EV_receiveR,0,8,EV_setC,0,10,EV_receiveNA,0,8,EV_receiveA,0,8,4,EV_receiveR,0,6,EV_receiveNA,0,9,EV_moveFlock,0,3,EV_receiveA,0,9,4,EV_receiveR,0,10,EV_receiveNA,0,10,EV_setF,0,0,EV_receiveA,0,10,2,EV_sendA,0,0,EV_condC1,0,1,3,EV_notCondC1,0,0,EV_sendA,0,1,EV_sendR,0,1,2,EV_sendA,0,0,EV_condC2,0,1,3,EV_notCondC2,0,0,EV_sendA,0,1,EV_sendR,0,1,2,EV_sendA,0,0,EV_condC3,0,1,3,EV_notCondC3,0,0,EV_sendR,0,1,EV_sendA,0,1,5,EV_receiveNA,0,0,EV_setC,0,1,EV_receiveR,0,0,EV_receiveA,0,0,EV_sendR,0,0,5,EV_receiveNA,0,1,EV_receiveR,0,2,EV_setF,0,0,EV_receiveA,0,1,EV_sendR,0,1,5,EV_receiveNA,0,2,EV_sendA,0,1,EV_receiveR,0,2,EV_receiveA,0,2,EV_sendR,0,2 };

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
    const unsigned long int sup_data_prob_pos[6] = { 0,11,33,38,43,48 };
    const float             sup_data_prob[ 57 ] = { 2,0.50000000,0.50000000,2,0.50000000,0.50000000,1,1,2,0.50000000,0.50000000,2,0.50000000,0.50000000,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,2,0.50000000,0.50000000,1,1,2,0.50000000,0.50000000,1,1,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000 };
    const unsigned long int sup_data_var_prob_pos[6] = { 0,7,18,21,24,27 };
    const unsigned char     sup_data_var_prob[ 33 ] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    float                   current_var_prob[0] = {  };

};

#endif