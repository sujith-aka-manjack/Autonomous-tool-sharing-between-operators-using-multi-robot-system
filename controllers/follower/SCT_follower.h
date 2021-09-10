#ifndef SCT_FOLLOWER_H
#define SCT_FOLLOWER_H

#include <stdlib.h>
#include <ctime>
#include <queue>
#include <map>
#include <functional>
#include <iostream>

#include <argos3/core/utility/math/rng.h>

namespace follower{

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

    virtual std::string get_current_state_string();

    /* Supervisor Info */
    // const static unsigned char NUM_EVENTS = 30;
    // const static unsigned char NUM_SUPERVISORS = 11;

    const static unsigned char NUM_EVENTS = 28;
    const static unsigned char NUM_SUPERVISORS = 10;

    /* Event Info */
    // const static unsigned char EV_moveStop = 0;
    // const static unsigned char EV_requestC = 1;
    // const static unsigned char EV_switchC = 2;
    // const static unsigned char EV_taskStart = 3;
    // const static unsigned char EV_requestL = 4;
    // const static unsigned char EV_relay = 5;
    // const static unsigned char EV_respond = 6;
    // const static unsigned char EV_taskStop = 7;
    // const static unsigned char EV_switchF = 8;
    // const static unsigned char EV_moveFlock = 9;
    // const static unsigned char EV_nearC = 10;
    // const static unsigned char EV_notNearC = 11;
    // const static unsigned char EV_notCondC1 = 12;
    // const static unsigned char EV_condC1 = 13;
    // const static unsigned char EV_notCondC2 = 14;
    // const static unsigned char EV_condC2 = 15;
    // const static unsigned char EV_notCondC3 = 16;
    // const static unsigned char EV_condC3 = 17;
    // const static unsigned char EV_notCondF1 = 18;
    // const static unsigned char EV_condF1 = 19;
    // const static unsigned char EV_notCondF2 = 20;
    // const static unsigned char EV_condF2 = 21;
    // const static unsigned char EV_accept = 22;
    // const static unsigned char EV__respond = 23;
    // const static unsigned char EV_reject = 24;
    // const static unsigned char EV__relay = 25;
    // const static unsigned char EV__message = 26;
    // const static unsigned char EV__requestC = 27;
    // const static unsigned char EV__start = 28;
    // const static unsigned char EV__stop = 29;

    const static unsigned char EV_moveStop = 0;
    const static unsigned char EV_requestC = 1;
    const static unsigned char EV_switchC = 2;
    const static unsigned char EV_taskStart = 3;
    const static unsigned char EV_requestL = 4;
    const static unsigned char EV_relay = 5;
    const static unsigned char EV_respond = 6;
    const static unsigned char EV_taskStop = 7;
    const static unsigned char EV_switchF = 8;
    const static unsigned char EV_moveFlock = 9;
    const static unsigned char EV_nearC = 10;
    const static unsigned char EV_notNearC = 11;
    const static unsigned char EV_notCondC1 = 12;
    const static unsigned char EV_condC1 = 13;
    const static unsigned char EV_notCondC2 = 14;
    const static unsigned char EV_condC2 = 15;
    const static unsigned char EV_notCondF1 = 16;
    const static unsigned char EV_condF1 = 17;
    const static unsigned char EV_notCondF2 = 18;
    const static unsigned char EV_condF2 = 19;
    const static unsigned char EV_accept = 20;
    const static unsigned char EV__respond = 21;
    const static unsigned char EV_reject = 22;
    const static unsigned char EV__relay = 23;
    const static unsigned char EV__message = 24;
    const static unsigned char EV__requestC = 25;
    const static unsigned char EV__start = 26;
    const static unsigned char EV__stop = 27;

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
    const unsigned char     ev_controllable[28] = { 1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    const unsigned char     sup_events[10][28] = { { 1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,1,0,0,1,1,1,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,1,0,0,1,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,1,0,0,1,1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0 },{ 0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0 },{ 1,1,1,0,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0 },{ 0,1,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1 },{ 0,1,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1 },{ 0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1 } };
    const unsigned long int sup_init_state[10]     = { 0,0,0,0,0,0,0,0,0,0 };
    unsigned long int       sup_current_state[10]  = { 0,0,0,0,0,0,0,0,0,0 };
    const unsigned long int sup_data_pos[10] = { 0,152,178,204,230,255,280,518,571,624 };
    const unsigned char     sup_data[ 694 ] = { 6,EV_requestC,0,0,EV_switchC,0,1,EV_taskStart,0,2,EV_requestL,0,0,EV_relay,0,0,EV_moveFlock,0,3,5,EV_requestC,0,1,EV_requestL,0,1,EV_relay,0,1,EV_respond,0,1,EV_switchF,0,0,6,EV_requestC,0,2,EV_switchC,0,4,EV_requestL,0,2,EV_relay,0,2,EV_taskStop,0,0,EV_moveFlock,0,5,6,EV_moveStop,0,0,EV_requestC,0,3,EV_switchC,0,6,EV_taskStart,0,5,EV_requestL,0,3,EV_relay,0,3,6,EV_requestC,0,4,EV_requestL,0,4,EV_relay,0,4,EV_respond,0,4,EV_taskStop,0,1,EV_switchF,0,2,6,EV_moveStop,0,2,EV_requestC,0,5,EV_switchC,0,7,EV_requestL,0,5,EV_relay,0,5,EV_taskStop,0,3,6,EV_moveStop,0,1,EV_requestC,0,6,EV_requestL,0,6,EV_relay,0,6,EV_respond,0,6,EV_switchF,0,3,7,EV_moveStop,0,4,EV_requestC,0,7,EV_requestL,0,7,EV_relay,0,7,EV_respond,0,7,EV_taskStop,0,6,EV_switchF,0,5,4,EV_nearC,0,1,EV_requestL,0,0,EV_respond,0,0,EV_relay,0,0,4,EV_requestC,0,1,EV_respond,0,1,EV_notNearC,0,0,EV_relay,0,1,3,EV_respond,0,0,EV_relay,0,0,EV_condC1,0,1,5,EV_notCondC1,0,0,EV_requestL,0,1,EV_requestC,0,1,EV_respond,0,1,EV_relay,0,1,3,EV_respond,0,0,EV_relay,0,0,EV_condC2,0,1,5,EV_notCondC2,0,0,EV_requestL,0,1,EV_requestC,0,1,EV_respond,0,1,EV_relay,0,1,2,EV_switchC,0,1,EV_condF1,0,2,1,EV_condF1,0,3,2,EV_switchC,0,3,EV_notCondF1,0,0,2,EV_notCondF1,0,1,EV_switchF,0,2,2,EV_switchC,0,1,EV_condF2,0,2,1,EV_condF2,0,3,2,EV_switchC,0,3,EV_notCondF2,0,0,2,EV_notCondF2,0,1,EV_switchF,0,2,4,EV__respond,0,1,EV_relay,0,0,EV_moveFlock,0,2,EV_respond,0,0,5,EV_accept,0,0,EV_reject,0,0,EV_relay,0,1,EV_moveFlock,0,3,EV_respond,0,1,5,EV__respond,0,3,EV_requestL,0,4,EV_requestC,0,4,EV_relay,0,2,EV_respond,0,2,6,EV_accept,0,2,EV_reject,0,2,EV_requestL,0,5,EV_requestC,0,5,EV_relay,0,3,EV_respond,0,3,4,EV_moveStop,0,6,EV__respond,0,5,EV_relay,0,4,EV_respond,0,4,5,EV_moveStop,0,7,EV_accept,0,8,EV_reject,0,2,EV_relay,0,5,EV_respond,0,5,3,EV__respond,0,7,EV_relay,0,6,EV_respond,0,6,4,EV_accept,0,9,EV_reject,0,0,EV_relay,0,7,EV_respond,0,7,5,EV_moveStop,0,9,EV_switchC,0,10,EV__respond,0,11,EV_relay,0,8,EV_respond,0,8,4,EV_switchC,0,12,EV__respond,0,13,EV_relay,0,9,EV_respond,0,9,4,EV_moveStop,0,12,EV__respond,0,14,EV_relay,0,10,EV_respond,0,10,6,EV_moveStop,0,13,EV_accept,0,8,EV_switchC,0,14,EV_reject,0,8,EV_relay,0,11,EV_respond,0,11,4,EV__respond,0,15,EV_relay,0,12,EV_switchF,0,0,EV_respond,0,12,5,EV_accept,0,9,EV_switchC,0,15,EV_reject,0,9,EV_relay,0,13,EV_respond,0,13,5,EV_moveStop,0,15,EV_accept,0,10,EV_reject,0,10,EV_relay,0,14,EV_respond,0,14,5,EV_accept,0,12,EV_reject,0,12,EV_relay,0,15,EV_switchF,0,1,EV_respond,0,15,8,EV__relay,0,0,EV_requestC,0,0,EV__message,0,0,EV__requestC,0,1,EV_requestL,0,0,EV__start,0,0,EV_relay,0,0,EV__stop,0,0,9,EV__relay,0,1,EV_requestC,0,1,EV__message,0,1,EV__requestC,0,1,EV_requestL,0,1,EV__start,0,1,EV_relay,0,1,EV_respond,0,0,EV__stop,0,1,8,EV__relay,0,1,EV_requestC,0,0,EV__message,0,1,EV__requestC,0,0,EV_requestL,0,0,EV__start,0,0,EV_respond,0,0,EV__stop,0,0,9,EV__relay,0,1,EV_requestC,0,1,EV__message,0,1,EV__requestC,0,1,EV_requestL,0,1,EV__start,0,1,EV_relay,0,0,EV_respond,0,1,EV__stop,0,1,5,EV__relay,0,0,EV__stop,0,0,EV__start,0,1,EV__requestC,0,0,EV__message,0,0,6,EV__relay,0,1,EV__stop,0,0,EV__start,0,1,EV_taskStart,0,2,EV__requestC,0,1,EV__message,0,1,5,EV__relay,0,2,EV__stop,0,3,EV__start,0,2,EV__requestC,0,2,EV__message,0,2,6,EV__relay,0,3,EV__stop,0,3,EV__start,0,2,EV_taskStop,0,0,EV__requestC,0,3,EV__message,0,3 };

    /* Random number generator */
    argos::CRandom::CRNG* m_pcRNG;

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
    const unsigned char     ev_shared[28] = { 0,1,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,1,1,1 };

};

}

#endif