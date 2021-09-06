#ifndef SCT_LEADER_EXCHANGE_H
#define SCT_LEADER_EXCHANGE_H

#include <stdlib.h>
#include <ctime>
#include <queue>
#include <map>
#include <functional>
#include <iostream>

#include <argos3/core/utility/math/rng.h>

namespace leader_exchange {

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
    const static unsigned char NUM_EVENTS = 12;
    const static unsigned char NUM_SUPERVISORS = 5;

    /* Event Info */
    const static unsigned char EV_inputExchange = 0;
    const static unsigned char EV_exchange = 1;
    const static unsigned char EV_message = 2;
    const static unsigned char EV_inputMessage = 3;
    const static unsigned char EV_stop = 4;
    const static unsigned char EV_start = 5;
    const static unsigned char EV_respond = 6;
    const static unsigned char EV_inputStart = 7;
    const static unsigned char EV_inputStop = 8;
    const static unsigned char EV__relay = 9;
    const static unsigned char EV__requestL = 10;
    const static unsigned char EV__message = 10;

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
    const unsigned char     ev_controllable[12] = { 0,1,1,0,1,1,1,0,0,0,0,0 };
    const unsigned char     sup_events[5][12] = { { 1,1,1,1,1,1,1,1,1,0,0,0 },{ 1,1,1,1,1,1,1,1,1,0,0,0 },{ 1,1,1,1,1,1,1,1,1,0,0,0 },{ 0,1,1,0,1,1,1,0,0,1,1,1 },{ 1,1,1,1,1,1,1,1,1,0,0,0 } };
    const unsigned long int sup_init_state[5]     = { 0,0,0,0,0 };
    unsigned long int       sup_current_state[5]  = { 0,0,0,0,0 };
    const unsigned long int sup_data_pos[5] = { 0,53,106,159,206 };
    const unsigned char     sup_data[ 259 ] = { 8,EV_inputExchange,0,0,EV_exchange,0,0,EV_message,0,0,EV_inputMessage,0,0,EV_stop,0,0,EV_respond,0,0,EV_inputStart,0,1,EV_inputStop,0,0,9,EV_inputExchange,0,1,EV_exchange,0,1,EV_message,0,1,EV_inputMessage,0,1,EV_stop,0,1,EV_start,0,0,EV_respond,0,1,EV_inputStart,0,1,EV_inputStop,0,1,8,EV_inputExchange,0,0,EV_exchange,0,0,EV_message,0,0,EV_inputMessage,0,0,EV_start,0,0,EV_respond,0,0,EV_inputStart,0,0,EV_inputStop,0,1,9,EV_inputExchange,0,1,EV_exchange,0,1,EV_message,0,1,EV_inputMessage,0,1,EV_stop,0,0,EV_start,0,1,EV_respond,0,1,EV_inputStart,0,1,EV_inputStop,0,1,8,EV_inputExchange,0,0,EV_exchange,0,0,EV_inputMessage,0,1,EV_stop,0,0,EV_start,0,0,EV_respond,0,0,EV_inputStart,0,0,EV_inputStop,0,0,9,EV_inputExchange,0,1,EV_exchange,0,1,EV_message,0,0,EV_inputMessage,0,1,EV_stop,0,1,EV_start,0,1,EV_respond,0,1,EV_inputStart,0,1,EV_inputStop,0,1,7,EV_message,0,0,EV__relay,0,0,EV_stop,0,0,EV__requestL,0,1,EV_start,0,0,EV_exchange,0,0,EV__message,0,0,8,EV_message,0,1,EV__relay,0,1,EV_stop,0,1,EV__requestL,0,1,EV_start,0,1,EV_exchange,0,1,EV_respond,0,0,EV__message,0,1,8,EV_inputExchange,0,1,EV_message,0,0,EV_inputMessage,0,0,EV_stop,0,0,EV_start,0,0,EV_respond,0,0,EV_inputStart,0,0,EV_inputStop,0,0,9,EV_inputExchange,0,1,EV_exchange,0,0,EV_message,0,1,EV_inputMessage,0,1,EV_stop,0,1,EV_start,0,1,EV_respond,0,1,EV_inputStart,0,1,EV_inputStop,0,1 };
    
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
    const unsigned char     ev_shared[12] = { 0,1,1,0,1,1,1,0,0,1,1,1 };

};

}

#endif