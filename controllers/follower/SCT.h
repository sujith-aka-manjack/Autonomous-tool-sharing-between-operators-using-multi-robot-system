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
#define NUM_EVENTS 19
#define NUM_SUPERVISORS 5

/* Event Info */
#define EV_assignC 0

#define EV_condC1 1

#define EV_notCondF1 2

#define EV_setF 3

#define EV_moveFlock 4

#define EV_condC2 5

#define EV_notCondC2 6

#define EV_notCondC1 7

#define EV_condF2 8

#define EV_condF1 9

#define EV_moveStop 10

#define EV_notCondF2 11

#define EV_assignF 12

#define EV_setC 13

#define EV_sendA 14

#define EV_receiveA 15

#define EV_receiveR 16

#define EV_sendR 17

#define EV_receiveNA 18


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
    const unsigned char     ev_controllable[19] = { 0,0,0,1,1,0,0,0,0,0,1,0,0,1,1,0,0,1,0 };
    const unsigned char     sup_events[5][19] = { { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0 },{ 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 },{ 1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,0,0,1,0 },{ 1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1 },{ 1,1,1,1,0,1,1,1,1,1,0,1,1,1,0,0,0,0,0 } };
    const unsigned long int sup_init_state[5]     = { 0,0,0,0,0 };
    unsigned long int       sup_current_state[5]  = { 0,0,0,0,0 };    
    const unsigned long int sup_data_pos[5] = { 0,80,323,491,594 };
    const unsigned char     sup_data[ 688 ] = { 2,EV_assignC,0,1,EV_assignF,0,2,5,EV_notCondF1,0,1,EV_setF,0,2,EV_condF2,0,1,EV_condF1,0,1,EV_notCondF2,0,1,6,EV_condC1,0,2,EV_moveFlock,0,3,EV_condC2,0,2,EV_notCondC2,0,2,EV_notCondC1,0,2,EV_setC,0,1,6,EV_condC1,0,3,EV_condC2,0,3,EV_notCondC2,0,3,EV_notCondC1,0,3,EV_moveStop,0,2,EV_setC,0,4,6,EV_notCondF1,0,4,EV_setF,0,3,EV_condF2,0,4,EV_condF1,0,4,EV_moveStop,0,1,EV_notCondF2,0,4,6,EV_sendA,0,0,EV_assignF,0,1,EV_receiveA,0,0,EV_receiveR,0,0,EV_assignC,0,2,EV_receiveNA,0,0,10,EV_sendA,0,1,EV_notCondC2,0,1,EV_moveFlock,0,3,EV_receiveA,0,1,EV_receiveR,0,1,EV_sendR,0,4,EV_notCondC1,0,1,EV_receiveNA,0,1,EV_condC2,0,1,EV_condC1,0,1,9,EV_sendA,0,2,EV_notCondF2,0,2,EV_notCondF1,0,2,EV_receiveA,0,2,EV_receiveR,0,2,EV_condF2,0,2,EV_condF1,0,2,EV_receiveNA,0,2,EV_setF,0,1,9,EV_sendA,0,3,EV_notCondC2,0,3,EV_receiveA,0,3,EV_receiveR,0,3,EV_sendR,0,5,EV_notCondC1,0,3,EV_receiveNA,0,3,EV_condC2,0,3,EV_condC1,0,3,9,EV_sendA,0,4,EV_notCondC2,0,4,EV_moveFlock,0,5,EV_receiveA,0,4,EV_receiveR,0,4,EV_notCondC1,0,4,EV_receiveNA,0,4,EV_condC2,0,4,EV_condC1,0,4,9,EV_moveStop,0,6,EV_sendA,0,5,EV_notCondC2,0,5,EV_receiveA,0,5,EV_receiveR,0,5,EV_notCondC1,0,5,EV_receiveNA,0,5,EV_condC2,0,5,EV_condC1,0,5,8,EV_sendA,0,6,EV_notCondC2,0,6,EV_receiveA,0,7,EV_receiveR,0,6,EV_notCondC1,0,6,EV_receiveNA,0,8,EV_condC2,0,6,EV_condC1,0,6,9,EV_sendA,0,7,EV_setC,0,2,EV_notCondC2,0,7,EV_receiveA,0,7,EV_receiveR,0,7,EV_notCondC1,0,7,EV_receiveNA,0,7,EV_condC2,0,7,EV_condC1,0,7,9,EV_sendA,0,8,EV_notCondC2,0,8,EV_moveFlock,0,3,EV_receiveA,0,8,EV_receiveR,0,8,EV_notCondC1,0,8,EV_receiveNA,0,8,EV_condC2,0,8,EV_condC1,0,8,3,EV_assignC,0,1,EV_sendA,0,0,EV_assignF,0,2,6,EV_sendA,0,1,EV_notCondF1,0,1,EV_condF2,0,1,EV_condF1,0,1,EV_notCondF2,0,1,EV_setF,0,2,6,EV_sendA,0,2,EV_condC2,0,3,EV_notCondC1,0,2,EV_notCondC2,0,2,EV_setC,0,1,EV_condC1,0,4,6,EV_sendA,0,3,EV_condC2,0,3,EV_notCondC1,0,3,EV_notCondC2,0,2,EV_setC,0,5,EV_condC1,0,6,6,EV_sendA,0,4,EV_condC2,0,6,EV_notCondC1,0,2,EV_notCondC2,0,4,EV_setC,0,7,EV_condC1,0,4,6,EV_sendA,0,5,EV_notCondF1,0,5,EV_condF2,0,5,EV_condF1,0,5,EV_notCondF2,0,5,EV_setF,0,3,7,EV_sendA,0,6,EV_condC2,0,6,EV_sendR,0,6,EV_notCondC1,0,3,EV_notCondC2,0,4,EV_setC,0,8,EV_condC1,0,6,6,EV_sendA,0,7,EV_notCondF1,0,7,EV_condF2,0,7,EV_condF1,0,7,EV_notCondF2,0,7,EV_setF,0,4,7,EV_sendA,0,8,EV_notCondF1,0,8,EV_sendR,0,8,EV_condF2,0,8,EV_condF1,0,8,EV_notCondF2,0,8,EV_setF,0,6,6,EV_assignC,0,1,EV_assignF,0,2,EV_receiveNA,0,0,EV_sendR,0,0,EV_receiveR,0,0,EV_receiveA,0,0,9,EV_notCondF2,0,1,EV_notCondF1,0,1,EV_receiveNA,0,1,EV_sendR,0,1,EV_receiveR,0,3,EV_condF2,0,1,EV_condF1,0,1,EV_setF,0,2,EV_receiveA,0,1,9,EV_setC,0,1,EV_notCondC2,0,2,EV_receiveNA,0,2,EV_sendR,0,2,EV_notCondC1,0,2,EV_receiveR,0,2,EV_condC2,0,2,EV_receiveA,0,2,EV_condC1,0,2,9,EV_sendA,0,1,EV_notCondF2,0,3,EV_notCondF1,0,3,EV_receiveNA,0,3,EV_sendR,0,3,EV_receiveR,0,3,EV_condF2,0,3,EV_condF1,0,3,EV_receiveA,0,3,2,EV_assignC,0,1,EV_assignF,0,2,4,EV_notCondF1,0,1,EV_condF2,0,3,EV_condF1,0,4,EV_notCondF2,0,1,5,EV_condC2,0,2,EV_notCondC1,0,2,EV_notCondC2,0,2,EV_setC,0,1,EV_condC1,0,2,4,EV_notCondF1,0,3,EV_condF2,0,3,EV_condF1,0,5,EV_notCondF2,0,1,4,EV_notCondF1,0,1,EV_condF2,0,5,EV_condF1,0,4,EV_notCondF2,0,4,5,EV_notCondF1,0,3,EV_condF2,0,5,EV_condF1,0,5,EV_notCondF2,0,4,EV_setF,0,6,5,EV_condC2,0,6,EV_notCondC1,0,6,EV_notCondC2,0,6,EV_setC,0,5,EV_condC1,0,6 };
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
    const unsigned long int sup_data_prob_pos[5] = { 0,12,38,66,77 };
    const float             sup_data_prob[ 87 ] = { 0,1,1,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000,1,1,3,0.33333333,0.33333333,0.33333333,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000,1,1,2,0.50000000,0.50000000,2,0.50000000,0.50000000,1,1,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000,3,0.33333333,0.33333333,0.33333333,2,0.50000000,0.50000000,3,0.33333333,0.33333333,0.33333333,1,1,2,0.50000000,0.50000000,2,0.50000000,0.50000000,2,0.50000000,0.50000000,0,0,1,1,0,0,1,1,1,1 };
    const unsigned long int sup_data_var_prob_pos[5] = { 0,7,24,43,50 };
    const unsigned char     sup_data_var_prob[ 53 ] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    float                   current_var_prob[0] = {  };
    };

#endif