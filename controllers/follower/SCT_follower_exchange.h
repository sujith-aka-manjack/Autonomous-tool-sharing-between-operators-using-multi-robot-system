#ifndef SCT_FOLLOWER_EXCHANGE_H
#define SCT_FOLLOWER_EXCHANGE_H

#include <stdlib.h>
#include <ctime>
#include <queue>
#include <map>
#include <functional>
#include <iostream>

#include <argos3/core/utility/math/rng.h>

namespace follower_exchange{

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
    const static unsigned char NUM_EVENTS = 35;
    const static unsigned char NUM_SUPERVISORS = 11;

    /* Event Info */
    const static unsigned char EV_moveStop = 0;
    const static unsigned char EV_requestC = 1;
    const static unsigned char EV_switchC = 2;
    const static unsigned char EV_taskStart = 3;
    const static unsigned char EV_relay = 4;
    const static unsigned char EV_moveTeam = 5;
    const static unsigned char EV_requestL = 6;
    const static unsigned char EV_switchT = 7;
    const static unsigned char EV_respond = 8;
    const static unsigned char EV_taskStop = 9;
    const static unsigned char EV_switchF = 10;
    const static unsigned char EV_moveFlock = 11;
    const static unsigned char EV_nearC = 12;
    const static unsigned char EV_notNearC = 13;
    const static unsigned char EV_notCondC1 = 14;
    const static unsigned char EV_condC1 = 15;
    const static unsigned char EV_notCondC2 = 16;
    const static unsigned char EV_condC2 = 17;
    const static unsigned char EV_notCondF1 = 18;
    const static unsigned char EV_condF1 = 19;
    const static unsigned char EV_condF2 = 20;
    const static unsigned char EV_notCondF2 = 21;
    const static unsigned char EV_accept = 22;
    const static unsigned char EV__respond = 23;
    const static unsigned char EV_reject = 24;
    const static unsigned char EV__relay = 25;
    const static unsigned char EV__message = 26;
    const static unsigned char EV__requestC = 27;
    const static unsigned char EV__start = 28;
    const static unsigned char EV__stop = 29;
    const static unsigned char EV_notChosen = 30;
    const static unsigned char EV_nearLF = 31;
    const static unsigned char EV__exchange = 32;
    const static unsigned char EV_chosen = 33;
    const static unsigned char EV_notNearLF = 34;

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
    const unsigned char     ev_controllable[35] = { 1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    const unsigned char     sup_events[11][35] = { { 1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,1,0,0,1,0,1,0,1,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,1,0,0,1,0,1,0,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,1,0,0,1,0,1,0,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 0,0,1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0 },{ 1,1,1,0,1,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0 },{ 0,1,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0 },{ 0,1,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0 },{ 0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0 },{ 1,0,1,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1 } };
    const unsigned long int sup_init_state[11]     = { 0,0,0,0,0,0,0,0,0,0,0 };
    unsigned long int       sup_current_state[11]  = { 0,0,0,0,0,0,0,0,0,0,0 };
    const unsigned long int sup_data_pos[11] = { 0,366,392,418,444,493,542,1475,1528,1581,1651 };
    const unsigned char     sup_data[ 2662 ] = { 7,EV_requestC,0,0,EV_switchC,0,1,EV_taskStart,0,2,EV_relay,0,0,EV_requestL,0,0,EV_switchT,0,3,EV_moveFlock,0,4,5,EV_requestC,0,1,EV_relay,0,1,EV_requestL,0,1,EV_respond,0,1,EV_switchF,0,0,7,EV_requestC,0,2,EV_switchC,0,5,EV_relay,0,2,EV_requestL,0,2,EV_switchT,0,6,EV_taskStop,0,0,EV_moveFlock,0,7,5,EV_requestC,0,3,EV_relay,0,3,EV_moveTeam,0,8,EV_requestL,0,3,EV_switchF,0,0,7,EV_moveStop,0,0,EV_requestC,0,4,EV_switchC,0,9,EV_taskStart,0,7,EV_relay,0,4,EV_requestL,0,4,EV_switchT,0,10,6,EV_requestC,0,5,EV_relay,0,5,EV_requestL,0,5,EV_respond,0,5,EV_taskStop,0,1,EV_switchF,0,2,6,EV_requestC,0,6,EV_relay,0,6,EV_moveTeam,0,11,EV_requestL,0,6,EV_taskStop,0,3,EV_switchF,0,2,7,EV_moveStop,0,2,EV_requestC,0,7,EV_switchC,0,12,EV_relay,0,7,EV_requestL,0,7,EV_switchT,0,13,EV_taskStop,0,4,5,EV_moveStop,0,3,EV_requestC,0,8,EV_relay,0,8,EV_requestL,0,8,EV_switchF,0,14,6,EV_moveStop,0,1,EV_requestC,0,9,EV_relay,0,9,EV_requestL,0,9,EV_respond,0,9,EV_switchF,0,4,6,EV_moveStop,0,3,EV_requestC,0,10,EV_relay,0,10,EV_moveTeam,0,8,EV_requestL,0,10,EV_switchF,0,4,6,EV_moveStop,0,6,EV_requestC,0,11,EV_relay,0,11,EV_requestL,0,11,EV_taskStop,0,8,EV_switchF,0,15,7,EV_moveStop,0,5,EV_requestC,0,12,EV_relay,0,12,EV_requestL,0,12,EV_respond,0,12,EV_taskStop,0,9,EV_switchF,0,7,7,EV_moveStop,0,6,EV_requestC,0,13,EV_relay,0,13,EV_moveTeam,0,11,EV_requestL,0,13,EV_taskStop,0,10,EV_switchF,0,7,8,EV_moveStop,0,0,EV_requestC,0,14,EV_switchC,0,16,EV_taskStart,0,15,EV_relay,0,14,EV_requestL,0,14,EV_switchT,0,8,EV_moveFlock,0,4,8,EV_moveStop,0,2,EV_requestC,0,15,EV_switchC,0,17,EV_relay,0,15,EV_requestL,0,15,EV_switchT,0,11,EV_taskStop,0,14,EV_moveFlock,0,7,6,EV_moveStop,0,1,EV_requestC,0,16,EV_relay,0,16,EV_requestL,0,16,EV_respond,0,16,EV_switchF,0,14,7,EV_moveStop,0,5,EV_requestC,0,17,EV_relay,0,17,EV_requestL,0,17,EV_respond,0,17,EV_taskStop,0,16,EV_switchF,0,15,4,EV_nearC,0,1,EV_requestL,0,0,EV_respond,0,0,EV_relay,0,0,4,EV_requestC,0,1,EV_respond,0,1,EV_notNearC,0,0,EV_relay,0,1,3,EV_respond,0,0,EV_relay,0,0,EV_condC1,0,1,5,EV_notCondC1,0,0,EV_requestL,0,1,EV_requestC,0,1,EV_respond,0,1,EV_relay,0,1,3,EV_respond,0,0,EV_relay,0,0,EV_condC2,0,1,5,EV_notCondC2,0,0,EV_requestL,0,1,EV_requestC,0,1,EV_respond,0,1,EV_relay,0,1,3,EV_switchT,0,1,EV_switchC,0,2,EV_condF1,0,3,2,EV_switchF,0,0,EV_condF1,0,4,1,EV_condF1,0,5,3,EV_switchT,0,4,EV_switchC,0,6,EV_notCondF1,0,0,2,EV_notCondF1,0,1,EV_switchF,0,3,2,EV_notCondF1,0,2,EV_switchF,0,3,1,EV_notCondF1,0,2,3,EV_condF2,0,1,EV_switchT,0,2,EV_switchC,0,3,3,EV_switchT,0,4,EV_switchC,0,5,EV_notCondF2,0,0,2,EV_condF2,0,4,EV_switchF,0,0,1,EV_condF2,0,6,2,EV_notCondF2,0,2,EV_switchF,0,1,1,EV_notCondF2,0,3,2,EV_notCondF2,0,3,EV_switchF,0,1,6,EV_moveFlock,0,1,EV__respond,0,2,EV_switchT,0,3,EV_respond,0,0,EV_moveTeam,0,4,EV_relay,0,0,7,EV_requestC,0,5,EV__respond,0,6,EV_requestL,0,5,EV_switchT,0,7,EV_respond,0,1,EV_moveTeam,0,8,EV_relay,0,1,7,EV_accept,0,0,EV_moveFlock,0,6,EV_reject,0,0,EV_switchT,0,9,EV_respond,0,2,EV_moveTeam,0,10,EV_relay,0,2,6,EV_moveFlock,0,11,EV__respond,0,9,EV_respond,0,3,EV_moveTeam,0,12,EV_switchF,0,0,EV_relay,0,3,5,EV_moveFlock,0,1,EV__respond,0,10,EV_switchT,0,12,EV_respond,0,4,EV_relay,0,4,5,EV_moveStop,0,13,EV__respond,0,14,EV_respond,0,5,EV_moveTeam,0,15,EV_relay,0,5,8,EV_accept,0,1,EV_requestC,0,14,EV_reject,0,1,EV_requestL,0,14,EV_switchT,0,16,EV_respond,0,6,EV_moveTeam,0,17,EV_relay,0,6,5,EV__respond,0,16,EV_respond,0,7,EV_moveTeam,0,12,EV_switchF,0,18,EV_relay,0,7,6,EV_requestC,0,15,EV__respond,0,17,EV_requestL,0,15,EV_switchT,0,12,EV_respond,0,8,EV_relay,0,8,7,EV_accept,0,3,EV_moveFlock,0,19,EV_reject,0,3,EV_respond,0,9,EV_moveTeam,0,20,EV_switchF,0,2,EV_relay,0,9,6,EV_accept,0,4,EV_moveFlock,0,6,EV_reject,0,4,EV_switchT,0,20,EV_respond,0,10,EV_relay,0,10,6,EV_requestC,0,21,EV__respond,0,19,EV_requestL,0,21,EV_respond,0,11,EV_moveTeam,0,22,EV_relay,0,11,5,EV_moveFlock,0,11,EV__respond,0,20,EV_respond,0,12,EV_switchF,0,4,EV_relay,0,12,4,EV__respond,0,23,EV_respond,0,13,EV_moveTeam,0,24,EV_relay,0,13,6,EV_moveStop,0,23,EV_accept,0,25,EV_reject,0,1,EV_respond,0,14,EV_moveTeam,0,26,EV_relay,0,14,4,EV_moveStop,0,13,EV__respond,0,26,EV_respond,0,15,EV_relay,0,15,6,EV_accept,0,7,EV_reject,0,7,EV_respond,0,16,EV_moveTeam,0,20,EV_switchF,0,27,EV_relay,0,16,7,EV_accept,0,8,EV_requestC,0,26,EV_reject,0,8,EV_requestL,0,26,EV_switchT,0,20,EV_respond,0,17,EV_relay,0,17,5,EV__respond,0,27,EV_switchT,0,7,EV_respond,0,18,EV_moveTeam,0,4,EV_relay,0,18,7,EV_accept,0,11,EV_requestC,0,28,EV_reject,0,11,EV_requestL,0,28,EV_respond,0,19,EV_moveTeam,0,29,EV_relay,0,19,6,EV_accept,0,12,EV_moveFlock,0,19,EV_reject,0,12,EV_respond,0,20,EV_switchF,0,10,EV_relay,0,20,5,EV_moveStop,0,30,EV__respond,0,28,EV_respond,0,21,EV_moveTeam,0,31,EV_relay,0,21,5,EV_requestC,0,31,EV__respond,0,29,EV_requestL,0,31,EV_respond,0,22,EV_relay,0,22,5,EV_accept,0,32,EV_reject,0,0,EV_respond,0,23,EV_moveTeam,0,33,EV_relay,0,23,3,EV__respond,0,33,EV_respond,0,24,EV_relay,0,24,6,EV_moveStop,0,32,EV_switchC,0,34,EV__respond,0,35,EV_respond,0,25,EV_moveTeam,0,36,EV_relay,0,25,5,EV_moveStop,0,23,EV_accept,0,36,EV_reject,0,8,EV_respond,0,26,EV_relay,0,26,6,EV_accept,0,18,EV_reject,0,18,EV_switchT,0,16,EV_respond,0,27,EV_moveTeam,0,10,EV_relay,0,27,6,EV_moveStop,0,37,EV_accept,0,38,EV_reject,0,11,EV_respond,0,28,EV_moveTeam,0,39,EV_relay,0,28,6,EV_accept,0,22,EV_requestC,0,39,EV_reject,0,22,EV_requestL,0,39,EV_respond,0,29,EV_relay,0,29,4,EV__respond,0,37,EV_respond,0,30,EV_moveTeam,0,40,EV_relay,0,30,4,EV_moveStop,0,30,EV__respond,0,39,EV_respond,0,31,EV_relay,0,31,5,EV_switchC,0,41,EV__respond,0,42,EV_respond,0,32,EV_moveTeam,0,36,EV_relay,0,32,4,EV_accept,0,36,EV_reject,0,4,EV_respond,0,33,EV_relay,0,33,6,EV_moveStop,0,41,EV__respond,0,43,EV_respond,0,34,EV_moveTeam,0,44,EV_switchF,0,18,EV_relay,0,34,7,EV_moveStop,0,42,EV_accept,0,25,EV_switchC,0,43,EV_reject,0,25,EV_respond,0,35,EV_moveTeam,0,45,EV_relay,0,35,5,EV_moveStop,0,32,EV_switchC,0,44,EV__respond,0,45,EV_respond,0,36,EV_relay,0,36,5,EV_accept,0,46,EV_reject,0,3,EV_respond,0,37,EV_moveTeam,0,47,EV_relay,0,37,5,EV_moveStop,0,46,EV__respond,0,48,EV_respond,0,38,EV_moveTeam,0,49,EV_relay,0,38,5,EV_moveStop,0,37,EV_accept,0,49,EV_reject,0,22,EV_respond,0,39,EV_relay,0,39,3,EV__respond,0,47,EV_respond,0,40,EV_relay,0,40,5,EV__respond,0,50,EV_respond,0,41,EV_moveTeam,0,44,EV_switchF,0,0,EV_relay,0,41,6,EV_accept,0,32,EV_switchC,0,50,EV_reject,0,32,EV_respond,0,42,EV_moveTeam,0,45,EV_relay,0,42,7,EV_moveStop,0,50,EV_accept,0,34,EV_reject,0,34,EV_respond,0,43,EV_moveTeam,0,51,EV_switchF,0,27,EV_relay,0,43,5,EV_moveStop,0,41,EV__respond,0,51,EV_respond,0,44,EV_switchF,0,4,EV_relay,0,44,6,EV_moveStop,0,42,EV_accept,0,36,EV_switchC,0,51,EV_reject,0,36,EV_respond,0,45,EV_relay,0,45,4,EV__respond,0,52,EV_respond,0,46,EV_moveTeam,0,49,EV_relay,0,46,4,EV_accept,0,49,EV_reject,0,12,EV_respond,0,47,EV_relay,0,47,6,EV_moveStop,0,52,EV_accept,0,38,EV_reject,0,38,EV_respond,0,48,EV_moveTeam,0,53,EV_relay,0,48,4,EV_moveStop,0,46,EV__respond,0,53,EV_respond,0,49,EV_relay,0,49,6,EV_accept,0,41,EV_reject,0,41,EV_respond,0,50,EV_moveTeam,0,51,EV_switchF,0,2,EV_relay,0,50,6,EV_moveStop,0,50,EV_accept,0,44,EV_reject,0,44,EV_respond,0,51,EV_switchF,0,10,EV_relay,0,51,5,EV_accept,0,46,EV_reject,0,46,EV_respond,0,52,EV_moveTeam,0,53,EV_relay,0,52,5,EV_moveStop,0,52,EV_accept,0,49,EV_reject,0,49,EV_respond,0,53,EV_relay,0,53,8,EV__relay,0,0,EV_requestC,0,0,EV__message,0,0,EV__requestC,0,1,EV_requestL,0,0,EV__start,0,0,EV_relay,0,0,EV__stop,0,0,9,EV__relay,0,1,EV_requestC,0,1,EV__message,0,1,EV__requestC,0,1,EV_requestL,0,1,EV__start,0,1,EV_relay,0,1,EV_respond,0,0,EV__stop,0,1,8,EV__relay,0,1,EV_requestC,0,0,EV__message,0,1,EV__requestC,0,0,EV_requestL,0,0,EV__start,0,0,EV_respond,0,0,EV__stop,0,0,9,EV__relay,0,1,EV_requestC,0,1,EV__message,0,1,EV__requestC,0,1,EV_requestL,0,1,EV__start,0,1,EV_relay,0,0,EV_respond,0,1,EV__stop,0,1,5,EV__relay,0,0,EV__stop,0,0,EV__start,0,1,EV__requestC,0,0,EV__message,0,0,6,EV__relay,0,1,EV__stop,0,0,EV__start,0,1,EV_taskStart,0,2,EV__requestC,0,1,EV__message,0,1,5,EV__relay,0,2,EV__stop,0,3,EV__start,0,2,EV__requestC,0,2,EV__message,0,2,6,EV__relay,0,3,EV__stop,0,3,EV__start,0,2,EV_taskStop,0,0,EV__requestC,0,3,EV__message,0,3,4,EV_switchC,0,1,EV_nearLF,0,2,EV__exchange,0,3,EV_moveFlock,0,4,4,EV_nearLF,0,5,EV__exchange,0,6,EV_moveFlock,0,7,EV_switchF,0,0,4,EV_switchC,0,5,EV__exchange,0,8,EV_moveFlock,0,9,EV_notNearLF,0,0,5,EV_notChosen,0,0,EV_switchC,0,6,EV_nearLF,0,8,EV_chosen,0,10,EV_moveFlock,0,11,4,EV_moveStop,0,0,EV_switchC,0,7,EV_nearLF,0,9,EV__exchange,0,11,4,EV__exchange,0,12,EV_moveFlock,0,13,EV_switchF,0,2,EV_notNearLF,0,1,5,EV_notChosen,0,1,EV_nearLF,0,12,EV_chosen,0,1,EV_moveFlock,0,14,EV_switchF,0,3,4,EV_moveStop,0,1,EV_nearLF,0,13,EV__exchange,0,14,EV_switchF,0,4,5,EV_notChosen,0,2,EV_switchC,0,12,EV_chosen,0,15,EV_moveFlock,0,16,EV_notNearLF,0,3,4,EV_moveStop,0,2,EV_switchC,0,13,EV__exchange,0,16,EV_notNearLF,0,4,5,EV_switchC,0,1,EV_nearLF,0,15,EV__exchange,0,17,EV_switchT,0,18,EV_moveFlock,0,19,5,EV_moveStop,0,3,EV_notChosen,0,4,EV_switchC,0,14,EV_nearLF,0,16,EV_chosen,0,19,5,EV_notChosen,0,5,EV_chosen,0,5,EV_moveFlock,0,20,EV_switchF,0,8,EV_notNearLF,0,6,4,EV_moveStop,0,5,EV__exchange,0,20,EV_switchF,0,9,EV_notNearLF,0,7,5,EV_moveStop,0,6,EV_notChosen,0,7,EV_nearLF,0,20,EV_chosen,0,7,EV_switchF,0,11,5,EV_switchC,0,5,EV__exchange,0,21,EV_switchT,0,22,EV_moveFlock,0,23,EV_notNearLF,0,10,5,EV_moveStop,0,8,EV_notChosen,0,9,EV_switchC,0,20,EV_chosen,0,23,EV_notNearLF,0,11,6,EV_notChosen,0,10,EV_switchC,0,6,EV_nearLF,0,21,EV_chosen,0,10,EV_switchT,0,24,EV_moveFlock,0,25,4,EV_nearLF,0,22,EV__exchange,0,24,EV_moveTeam,0,26,EV_moveFlock,0,27,5,EV_moveStop,0,10,EV_switchC,0,7,EV_nearLF,0,23,EV__exchange,0,25,EV_switchT,0,27,5,EV_moveStop,0,12,EV_notChosen,0,13,EV_chosen,0,13,EV_switchF,0,16,EV_notNearLF,0,14,6,EV_notChosen,0,15,EV_switchC,0,12,EV_chosen,0,15,EV_switchT,0,28,EV_moveFlock,0,29,EV_notNearLF,0,17,4,EV__exchange,0,28,EV_moveTeam,0,30,EV_moveFlock,0,31,EV_notNearLF,0,18,5,EV_moveStop,0,15,EV_switchC,0,13,EV__exchange,0,29,EV_switchT,0,31,EV_notNearLF,0,19,5,EV_notChosen,0,18,EV_nearLF,0,28,EV_chosen,0,18,EV_moveTeam,0,32,EV_moveFlock,0,33,6,EV_moveStop,0,17,EV_notChosen,0,19,EV_switchC,0,14,EV_nearLF,0,29,EV_chosen,0,19,EV_switchT,0,33,4,EV_moveStop,0,34,EV_nearLF,0,35,EV__exchange,0,32,EV_moveFlock,0,36,4,EV_moveStop,0,18,EV_nearLF,0,31,EV__exchange,0,33,EV_moveTeam,0,26,5,EV_notChosen,0,22,EV_chosen,0,22,EV_moveTeam,0,37,EV_moveFlock,0,38,EV_notNearLF,0,24,6,EV_moveStop,0,21,EV_notChosen,0,23,EV_switchC,0,20,EV_chosen,0,23,EV_switchT,0,38,EV_notNearLF,0,25,4,EV_moveStop,0,39,EV__exchange,0,37,EV_moveFlock,0,40,EV_notNearLF,0,26,4,EV_moveStop,0,22,EV__exchange,0,38,EV_moveTeam,0,30,EV_notNearLF,0,27,5,EV_moveStop,0,41,EV_notChosen,0,26,EV_nearLF,0,42,EV_chosen,0,26,EV_moveFlock,0,43,5,EV_moveStop,0,24,EV_notChosen,0,27,EV_nearLF,0,38,EV_chosen,0,27,EV_moveTeam,0,32,3,EV_nearLF,0,44,EV__exchange,0,41,EV_moveFlock,0,36,5,EV_moveStop,0,44,EV__exchange,0,42,EV_moveFlock,0,45,EV_switchF,0,46,EV_notNearLF,0,26,3,EV_moveStop,0,34,EV_nearLF,0,45,EV__exchange,0,43,5,EV_moveStop,0,47,EV_notChosen,0,30,EV_chosen,0,30,EV_moveFlock,0,48,EV_notNearLF,0,32,5,EV_moveStop,0,28,EV_notChosen,0,31,EV_chosen,0,31,EV_moveTeam,0,37,EV_notNearLF,0,33,3,EV__exchange,0,47,EV_moveFlock,0,40,EV_notNearLF,0,34,3,EV_moveStop,0,39,EV__exchange,0,48,EV_notNearLF,0,36,4,EV_notChosen,0,34,EV_nearLF,0,49,EV_chosen,0,34,EV_moveFlock,0,43,6,EV_moveStop,0,49,EV_notChosen,0,35,EV_chosen,0,35,EV_moveFlock,0,50,EV_switchF,0,51,EV_notNearLF,0,32,4,EV_moveStop,0,41,EV_notChosen,0,36,EV_nearLF,0,50,EV_chosen,0,36,4,EV__exchange,0,49,EV_moveFlock,0,45,EV_switchF,0,2,EV_notNearLF,0,34,4,EV_moveStop,0,44,EV__exchange,0,50,EV_switchF,0,9,EV_notNearLF,0,36,5,EV_moveStop,0,2,EV_switchC,0,52,EV__exchange,0,51,EV_moveFlock,0,9,EV_notNearLF,0,53,4,EV_notChosen,0,39,EV_chosen,0,39,EV_moveFlock,0,48,EV_notNearLF,0,41,4,EV_moveStop,0,47,EV_notChosen,0,40,EV_chosen,0,40,EV_notNearLF,0,43,5,EV_notChosen,0,44,EV_chosen,0,44,EV_moveFlock,0,50,EV_switchF,0,8,EV_notNearLF,0,41,5,EV_moveStop,0,49,EV_notChosen,0,45,EV_chosen,0,45,EV_switchF,0,16,EV_notNearLF,0,43,6,EV_moveStop,0,8,EV_notChosen,0,46,EV_switchC,0,54,EV_chosen,0,55,EV_moveFlock,0,16,EV_notNearLF,0,56,5,EV_moveStop,0,5,EV__exchange,0,54,EV_moveFlock,0,13,EV_switchF,0,46,EV_notNearLF,0,57,5,EV_moveStop,0,0,EV_switchC,0,57,EV_nearLF,0,46,EV__exchange,0,56,EV_moveFlock,0,4,6,EV_moveStop,0,12,EV_notChosen,0,52,EV_chosen,0,52,EV_moveFlock,0,20,EV_switchF,0,51,EV_notNearLF,0,58,6,EV_moveStop,0,15,EV_switchC,0,52,EV__exchange,0,59,EV_switchT,0,60,EV_moveFlock,0,23,EV_notNearLF,0,61,6,EV_moveStop,0,3,EV_notChosen,0,53,EV_switchC,0,58,EV_nearLF,0,51,EV_chosen,0,61,EV_moveFlock,0,11,5,EV_moveStop,0,1,EV_nearLF,0,52,EV__exchange,0,58,EV_moveFlock,0,7,EV_switchF,0,53,6,EV_moveStop,0,6,EV_notChosen,0,57,EV_nearLF,0,54,EV_chosen,0,57,EV_moveFlock,0,14,EV_switchF,0,56,7,EV_moveStop,0,21,EV_notChosen,0,55,EV_switchC,0,54,EV_chosen,0,55,EV_switchT,0,62,EV_moveFlock,0,29,EV_notNearLF,0,63,4,EV_moveStop,0,22,EV__exchange,0,62,EV_moveFlock,0,31,EV_notNearLF,0,64,6,EV_moveStop,0,10,EV_switchC,0,57,EV_nearLF,0,55,EV__exchange,0,63,EV_switchT,0,64,EV_moveFlock,0,19,5,EV_moveStop,0,28,EV_notChosen,0,60,EV_chosen,0,60,EV_moveFlock,0,38,EV_notNearLF,0,65,7,EV_moveStop,0,17,EV_notChosen,0,61,EV_switchC,0,58,EV_nearLF,0,59,EV_chosen,0,61,EV_switchT,0,65,EV_moveFlock,0,25,4,EV_moveStop,0,18,EV_nearLF,0,60,EV__exchange,0,65,EV_moveFlock,0,27,5,EV_moveStop,0,24,EV_notChosen,0,64,EV_nearLF,0,62,EV_chosen,0,64,EV_moveFlock,0,33 };

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
    const unsigned char     ev_shared[35] = { 0,1,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1,1,1,1,0,0,1,0,0 };

};

}

#endif