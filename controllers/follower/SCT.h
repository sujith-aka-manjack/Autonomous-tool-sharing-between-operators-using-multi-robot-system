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
#define NUM_EVENTS 31
#define NUM_SUPERVISORS 11

/* Event Info */
#define EV_moveStop 0

#define EV_taskBegin 1

#define EV_taskEnded 2

#define EV_moveFlock 3

#define EV_taskStop 4

#define EV_setF 5

#define EV_setC 6

#define EV_nearC 7

#define EV_sendReqL 8

#define EV_sendReqC 9

#define EV_sendReply 10

#define EV_relayMsg 11

#define EV_notNearC 12

#define EV_notCondC1 13

#define EV_condC1 14

#define EV_notCondC2 15

#define EV_condC2 16

#define EV_notCondC3 17

#define EV_condC3 18

#define EV_notCondF1 19

#define EV_condF1 20

#define EV_notCondF2 21

#define EV_condF2 22

#define EV__relayMsg 23

#define EV__sendBegin 24

#define EV__sendReply 25

#define EV__sendStop 26

#define EV_reject 27

#define EV__sendMsg 28

#define EV__sendReqC 29

#define EV_accept 30

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
    const unsigned char     ev_controllable[31] = { 1,1,0,1,1,1,1,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    const unsigned char     sup_events[11][31] = { { 1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0 },{ 0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0 },{ 1,0,0,1,0,1,1,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1 },{ 0,0,0,0,0,1,1,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0 },{ 0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0 },{ 0,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0 } };
    const unsigned long int sup_init_state[11]     = { 0,0,0,0,0,0,0,0,0,0,0 };
    unsigned long int       sup_current_state[11]  = { 0,0,0,0,0,0,0,0,0,0,0 };
    const unsigned long int sup_data_pos[11] = { 0,80,106,132,158,184,209,234,585,669,722 };
    const unsigned char     sup_data[ 798 ] = { 3,EV_taskBegin,0,1,EV_moveFlock,0,2,EV_setC,0,3,4,EV_taskEnded,0,0,EV_moveFlock,0,4,EV_taskStop,0,0,EV_setC,0,5,3,EV_moveStop,0,0,EV_taskBegin,0,4,EV_setC,0,6,1,EV_setF,0,0,4,EV_moveStop,0,1,EV_taskEnded,0,2,EV_taskStop,0,2,EV_setC,0,7,3,EV_taskEnded,0,3,EV_taskStop,0,3,EV_setF,0,1,2,EV_moveStop,0,3,EV_setF,0,2,4,EV_moveStop,0,5,EV_taskEnded,0,6,EV_taskStop,0,6,EV_setF,0,4,4,EV_nearC,0,1,EV_sendReqL,0,0,EV_sendReply,0,0,EV_relayMsg,0,0,4,EV_sendReqC,0,1,EV_sendReply,0,1,EV_relayMsg,0,1,EV_notNearC,0,0,3,EV_sendReply,0,0,EV_relayMsg,0,0,EV_condC1,0,1,5,EV_notCondC1,0,0,EV_sendReqL,0,1,EV_sendReqC,0,1,EV_sendReply,0,1,EV_relayMsg,0,1,3,EV_sendReply,0,0,EV_relayMsg,0,0,EV_condC2,0,1,5,EV_notCondC2,0,0,EV_sendReqL,0,1,EV_sendReqC,0,1,EV_sendReply,0,1,EV_relayMsg,0,1,3,EV_relayMsg,0,0,EV_sendReply,0,0,EV_condC3,0,1,5,EV_sendReqL,0,1,EV_relayMsg,0,1,EV_sendReqC,0,1,EV_sendReply,0,1,EV_notCondC3,0,0,2,EV_setC,0,1,EV_condF1,0,2,1,EV_condF1,0,3,2,EV_setC,0,3,EV_notCondF1,0,0,2,EV_notCondF1,0,1,EV_setF,0,2,2,EV_condF2,0,1,EV_setC,0,2,2,EV_notCondF2,0,0,EV_setC,0,3,1,EV_condF2,0,3,2,EV_setF,0,1,EV_notCondF2,0,2,9,EV__relayMsg,0,0,EV_moveFlock,0,1,EV__sendBegin,0,0,EV_sendReply,0,0,EV__sendReply,0,2,EV__sendStop,0,0,EV_relayMsg,0,0,EV__sendMsg,0,0,EV__sendReqC,0,0,10,EV__relayMsg,0,1,EV__sendBegin,0,1,EV_sendReply,0,1,EV__sendReply,0,3,EV__sendStop,0,1,EV_relayMsg,0,1,EV_sendReqL,0,4,EV__sendMsg,0,1,EV_sendReqC,0,4,EV__sendReqC,0,1,10,EV__relayMsg,0,2,EV_moveFlock,0,3,EV__sendBegin,0,2,EV_sendReply,0,2,EV__sendStop,0,2,EV_relayMsg,0,2,EV_reject,0,0,EV__sendMsg,0,2,EV__sendReqC,0,2,EV_accept,0,0,11,EV__relayMsg,0,3,EV__sendBegin,0,3,EV_sendReply,0,3,EV__sendStop,0,3,EV_relayMsg,0,3,EV_reject,0,1,EV_sendReqL,0,5,EV__sendMsg,0,3,EV_sendReqC,0,5,EV__sendReqC,0,3,EV_accept,0,1,9,EV_moveStop,0,6,EV__relayMsg,0,4,EV__sendBegin,0,4,EV_sendReply,0,4,EV__sendReply,0,5,EV__sendStop,0,4,EV_relayMsg,0,4,EV__sendMsg,0,4,EV__sendReqC,0,4,9,EV__relayMsg,0,5,EV__sendBegin,0,5,EV_sendReply,0,5,EV__sendStop,0,5,EV_relayMsg,0,5,EV_reject,0,4,EV__sendMsg,0,5,EV__sendReqC,0,5,EV_accept,0,4,8,EV__relayMsg,0,6,EV__sendBegin,0,6,EV_sendReply,0,6,EV__sendReply,0,7,EV__sendStop,0,6,EV_relayMsg,0,6,EV__sendMsg,0,6,EV__sendReqC,0,6,9,EV__relayMsg,0,7,EV__sendBegin,0,7,EV_sendReply,0,7,EV__sendStop,0,7,EV_relayMsg,0,7,EV_reject,0,0,EV__sendMsg,0,7,EV__sendReqC,0,7,EV_accept,0,8,9,EV__relayMsg,0,8,EV_setC,0,9,EV__sendBegin,0,8,EV_sendReply,0,8,EV__sendReply,0,10,EV__sendStop,0,8,EV_relayMsg,0,8,EV__sendMsg,0,8,EV__sendReqC,0,8,9,EV__relayMsg,0,9,EV__sendBegin,0,9,EV_sendReply,0,9,EV__sendReply,0,11,EV__sendStop,0,9,EV_relayMsg,0,9,EV__sendMsg,0,9,EV__sendReqC,0,9,EV_setF,0,0,10,EV__relayMsg,0,10,EV_setC,0,11,EV__sendBegin,0,10,EV_sendReply,0,10,EV__sendStop,0,10,EV_relayMsg,0,10,EV_reject,0,8,EV__sendMsg,0,10,EV__sendReqC,0,10,EV_accept,0,8,10,EV__relayMsg,0,11,EV__sendBegin,0,11,EV_sendReply,0,11,EV__sendStop,0,11,EV_relayMsg,0,11,EV_reject,0,9,EV__sendMsg,0,11,EV__sendReqC,0,11,EV_setF,0,2,EV_accept,0,9,9,EV__sendReqC,0,0,EV__sendBegin,0,0,EV_setC,0,1,EV_relayMsg,0,0,EV__sendMsg,0,0,EV_sendReqC,0,0,EV_sendReqL,0,0,EV__relayMsg,0,0,EV__sendStop,0,0,9,EV__sendReqC,0,2,EV_setF,0,0,EV__sendBegin,0,1,EV_relayMsg,0,1,EV__sendMsg,0,1,EV_sendReqC,0,1,EV_sendReqL,0,1,EV__relayMsg,0,1,EV__sendStop,0,1,9,EV__sendReqC,0,2,EV_sendReply,0,1,EV__sendBegin,0,2,EV_relayMsg,0,2,EV__sendMsg,0,2,EV_sendReqC,0,2,EV_sendReqL,0,2,EV__relayMsg,0,2,EV__sendStop,0,2,8,EV__sendReqC,0,0,EV_sendReply,0,0,EV_sendReqL,0,0,EV__sendMsg,0,1,EV_sendReqC,0,0,EV__sendStop,0,0,EV__relayMsg,0,1,EV__sendBegin,0,0,9,EV__sendReqC,0,1,EV_sendReply,0,1,EV_sendReqL,0,1,EV_relayMsg,0,0,EV__sendMsg,0,1,EV_sendReqC,0,1,EV__sendStop,0,1,EV__relayMsg,0,1,EV__sendBegin,0,1,5,EV__sendStop,0,0,EV__sendReqC,0,0,EV__sendMsg,0,0,EV__relayMsg,0,0,EV__sendBegin,0,1,6,EV__sendStop,0,0,EV_taskBegin,0,2,EV__sendReqC,0,1,EV__sendMsg,0,1,EV__relayMsg,0,1,EV__sendBegin,0,1,6,EV_taskEnded,0,1,EV__sendStop,0,3,EV__sendReqC,0,2,EV__sendMsg,0,2,EV__relayMsg,0,2,EV__sendBegin,0,2,7,EV_taskEnded,0,0,EV__sendStop,0,3,EV__sendReqC,0,3,EV__sendMsg,0,3,EV_taskStop,0,0,EV__relayMsg,0,3,EV__sendBegin,0,2 };

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
    const unsigned char     ev_shared[31] = { 0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,1,1,0 };

};

#endif