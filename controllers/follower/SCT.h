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
#define NUM_EVENTS 27
#define NUM_SUPERVISORS 10

/* Event Info */
#define EV_moveStop 0

#define EV_moveFlock 1

#define EV_setF 2

#define EV_setC 3

#define EV_nearC 4

#define EV_sendA 5

#define EV_notNearC 6

#define EV_sendRC 7

#define EV_sendRL 8

#define EV_notCondC1 9

#define EV_condC1 10

#define EV_notCondC2 11

#define EV_condC2 12

#define EV_notCondC3 13

#define EV_condC3 14

#define EV_notCondF1 15

#define EV_condF1 16

#define EV_notCondF2 17

#define EV_condF2 18

#define EV_receiveNA 19

#define EV_receiveA 20

#define EV_receiveTB 21

#define EV_receiveR 22

#define EV_receiveTS 23

#define EV_taskBegin 24

#define EV_taskEnded 25

#define EV_taskStop 26


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
    const unsigned char     ev_controllable[27] = { 1,1,1,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1 };
    const unsigned char     sup_events[10][27] = { { 1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,1,0,1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,1,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0 },{ 0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0 },{ 1,1,1,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0 },{ 0,0,1,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0 },{ 0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1 } };
    const unsigned long int sup_init_state[10]     = { 0,0,0,0,0,0,0,0,0,0 };
    unsigned long int       sup_current_state[10]  = { 0,0,0,0,0,0,0,0,0,0 };    
    const unsigned long int sup_data_pos[10] = { 0,25,45,65,85,105,130,155,287,362 };
    const unsigned char     sup_data[ 494 ] = { 2,EV_moveFlock,0,1,EV_setC,0,2,2,EV_moveStop,0,0,EV_setC,0,3,1,EV_setF,0,0,2,EV_moveStop,0,2,EV_setF,0,1,3,EV_nearC,0,1,EV_sendA,0,0,EV_sendRL,0,0,3,EV_sendA,0,1,EV_notNearC,0,0,EV_sendRC,0,1,2,EV_condC1,0,1,EV_sendA,0,0,4,EV_notCondC1,0,0,EV_sendA,0,1,EV_sendRC,0,1,EV_sendRL,0,1,2,EV_condC2,0,1,EV_sendA,0,0,4,EV_notCondC2,0,0,EV_sendA,0,1,EV_sendRC,0,1,EV_sendRL,0,1,2,EV_sendA,0,0,EV_condC3,0,1,4,EV_sendA,0,1,EV_notCondC3,0,0,EV_sendRL,0,1,EV_sendRC,0,1,2,EV_setC,0,1,EV_condF1,0,2,1,EV_condF1,0,3,2,EV_setC,0,3,EV_notCondF1,0,0,2,EV_notCondF1,0,1,EV_setF,0,2,2,EV_condF2,0,1,EV_setC,0,2,2,EV_notCondF2,0,0,EV_setC,0,3,1,EV_condF2,0,3,2,EV_setF,0,1,EV_notCondF2,0,2,7,EV_receiveNA,0,0,EV_sendA,0,0,EV_moveFlock,0,1,EV_receiveA,0,0,EV_receiveTB,0,0,EV_receiveR,0,0,EV_receiveTS,0,0,8,EV_receiveNA,0,1,EV_sendA,0,1,EV_sendRL,0,2,EV_sendRC,0,2,EV_receiveA,0,1,EV_receiveTB,0,1,EV_receiveR,0,1,EV_receiveTS,0,1,7,EV_receiveNA,0,2,EV_sendA,0,2,EV_receiveA,0,2,EV_receiveTB,0,2,EV_receiveR,0,2,EV_moveStop,0,3,EV_receiveTS,0,2,6,EV_receiveNA,0,0,EV_sendA,0,3,EV_receiveA,0,4,EV_receiveTB,0,3,EV_receiveR,0,3,EV_receiveTS,0,3,7,EV_receiveNA,0,4,EV_sendA,0,4,EV_setC,0,5,EV_receiveA,0,4,EV_receiveTB,0,4,EV_receiveR,0,4,EV_receiveTS,0,4,7,EV_receiveNA,0,5,EV_sendA,0,5,EV_receiveA,0,5,EV_receiveTB,0,5,EV_receiveR,0,5,EV_setF,0,0,EV_receiveTS,0,5,8,EV_receiveNA,0,0,EV_setC,0,1,EV_sendRL,0,0,EV_receiveA,0,0,EV_receiveTB,0,0,EV_receiveR,0,0,EV_sendRC,0,0,EV_receiveTS,0,0,8,EV_receiveNA,0,1,EV_sendRL,0,1,EV_receiveA,0,1,EV_receiveTB,0,1,EV_setF,0,0,EV_receiveR,0,2,EV_sendRC,0,1,EV_receiveTS,0,1,8,EV_receiveNA,0,2,EV_sendA,0,1,EV_sendRL,0,2,EV_receiveA,0,2,EV_receiveTB,0,2,EV_receiveR,0,2,EV_sendRC,0,2,EV_receiveTS,0,2,6,EV_receiveNA,0,0,EV_receiveA,0,0,EV_receiveR,0,0,EV_receiveTB,0,1,EV_setC,0,2,EV_receiveTS,0,0,7,EV_receiveNA,0,1,EV_taskBegin,0,3,EV_receiveA,0,1,EV_receiveR,0,1,EV_receiveTB,0,1,EV_setC,0,2,EV_receiveTS,0,0,6,EV_receiveNA,0,2,EV_receiveA,0,2,EV_receiveR,0,2,EV_receiveTB,0,2,EV_setF,0,0,EV_receiveTS,0,2,7,EV_receiveNA,0,3,EV_taskEnded,0,1,EV_receiveA,0,3,EV_receiveR,0,3,EV_receiveTB,0,3,EV_setC,0,4,EV_receiveTS,0,5,8,EV_receiveNA,0,4,EV_taskEnded,0,2,EV_receiveA,0,4,EV_receiveR,0,4,EV_receiveTB,0,4,EV_taskStop,0,2,EV_setF,0,5,EV_receiveTS,0,4,8,EV_receiveNA,0,5,EV_taskEnded,0,0,EV_receiveA,0,5,EV_receiveR,0,5,EV_receiveTB,0,3,EV_setC,0,4,EV_taskStop,0,0,EV_receiveTS,0,5 };

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
    const unsigned long int sup_data_prob_pos[10] = { 0,11,17,23,29,35,42,49,67,79 };
    const float             sup_data_prob[ 94 ] = { 2,0.50000000,0.50000000,2,0.50000000,0.50000000,1,1,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000,1,1,3,0.33333333,0.33333333,0.33333333,1,1,3,0.33333333,0.33333333,0.33333333,1,1,3,0.33333333,0.33333333,0.33333333,1,1,0,1,1,1,1,1,1,1,1,0,1,1,2,0.50000000,0.50000000,3,0.33333333,0.33333333,0.33333333,2,0.50000000,0.50000000,1,1,2,0.50000000,0.50000000,2,0.50000000,0.50000000,3,0.33333333,0.33333333,0.33333333,3,0.33333333,0.33333333,0.33333333,3,0.33333333,0.33333333,0.33333333,1,1,2,0.50000000,0.50000000,1,1,1,1,2,0.50000000,0.50000000,2,0.50000000,0.50000000 };
    const unsigned long int sup_data_var_prob_pos[10] = { 0,7,11,15,19,23,26,29,41,50 };
    const unsigned char     sup_data_var_prob[ 59 ] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    float                   current_var_prob[0] = {  };

};

#endif