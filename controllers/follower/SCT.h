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
#define EV_flock 0

#define EV_stopTask 1

#define EV_chainFar 2

#define EV_leaderFar 3

#define EV_startTask 4

#define EV_joinLeader 5

#define EV_taskEnded 6

#define EV_leaderNear 7

#define EV_chainNear 8

#define EV_joinChain 9

#define EV_stop 10

#define EV_getStop 11

#define EV_getStart 12

#define EV_closestToChain 13

#define EV_notClosestToChain 14

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
    const unsigned char     ev_controllable[15] = { 1,1,0,0,1,1,0,0,0,1,1,0,0,0,0 };
    const unsigned char     sup_events[6][15] = { { 1,1,1,1,1,1,1,1,1,1,1,0,0,0,0 },{ 0,1,0,0,1,0,1,0,0,0,0,1,1,0,0 },{ 0,0,1,1,0,1,0,1,1,1,0,0,0,0,0 },{ 0,0,1,1,0,1,0,1,1,1,0,0,0,1,1 },{ 0,0,1,1,0,1,0,1,1,1,0,0,0,0,0 },{ 0,0,1,1,0,1,0,1,1,1,0,0,0,1,1 } };
    const unsigned long int sup_init_state[6]     = { 0,0,0,0,0,0 };
    unsigned long int       sup_current_state[6]  = { 0,0,0,0,0,0 };    
    const unsigned long int sup_data_pos[6] = { 0,104,144,171,232,269 };
    const unsigned char     sup_data[ 330 ] = { 4,EV_flock,0,1,EV_chainFar,0,0,EV_startTask,0,2,EV_chainNear,0,0,4,EV_chainFar,0,1,EV_startTask,0,3,EV_chainNear,0,1,EV_joinChain,0,5,5,EV_flock,0,3,EV_stopTask,0,0,EV_chainFar,0,2,EV_taskEnded,0,0,EV_chainNear,0,2,5,EV_stopTask,0,1,EV_chainFar,0,3,EV_taskEnded,0,1,EV_chainNear,0,3,EV_joinChain,0,4,4,EV_leaderFar,0,4,EV_taskEnded,0,5,EV_leaderNear,0,4,EV_stop,0,6,3,EV_leaderFar,0,5,EV_leaderNear,0,5,EV_stop,0,7,4,EV_leaderFar,0,6,EV_joinLeader,0,2,EV_taskEnded,0,7,EV_leaderNear,0,6,3,EV_leaderFar,0,7,EV_joinLeader,0,0,EV_leaderNear,0,7,2,EV_getStop,0,0,EV_getStart,0,1,3,EV_getStop,0,0,EV_startTask,0,2,EV_getStart,0,1,3,EV_taskEnded,0,1,EV_getStop,0,3,EV_getStart,0,2,4,EV_stopTask,0,0,EV_taskEnded,0,0,EV_getStop,0,3,EV_getStart,0,2,2,EV_chainFar,0,1,EV_chainNear,0,0,3,EV_chainFar,0,1,EV_chainNear,0,0,EV_joinChain,0,2,3,EV_joinLeader,0,1,EV_leaderNear,0,2,EV_leaderFar,0,2,4,EV_closestToChain,0,1,EV_chainFar,0,0,EV_chainNear,0,0,EV_notClosestToChain,0,0,5,EV_closestToChain,0,1,EV_chainFar,0,1,EV_chainNear,0,1,EV_joinChain,0,2,EV_notClosestToChain,0,0,5,EV_joinLeader,0,1,EV_leaderNear,0,2,EV_closestToChain,0,2,EV_leaderFar,0,2,EV_notClosestToChain,0,3,5,EV_joinLeader,0,0,EV_leaderNear,0,3,EV_closestToChain,0,2,EV_leaderFar,0,3,EV_notClosestToChain,0,3,3,EV_chainFar,0,0,EV_chainNear,0,0,EV_joinChain,0,1,2,EV_leaderNear,0,2,EV_leaderFar,0,1,3,EV_joinLeader,0,3,EV_leaderNear,0,2,EV_leaderFar,0,1,3,EV_chainFar,0,3,EV_chainNear,0,3,EV_joinChain,0,2,5,EV_closestToChain,0,0,EV_chainFar,0,0,EV_chainNear,0,0,EV_joinChain,0,1,EV_notClosestToChain,0,2,4,EV_leaderNear,0,1,EV_closestToChain,0,1,EV_leaderFar,0,1,EV_notClosestToChain,0,3,5,EV_closestToChain,0,0,EV_chainFar,0,2,EV_chainNear,0,2,EV_joinChain,0,3,EV_notClosestToChain,0,2,5,EV_joinLeader,0,2,EV_leaderNear,0,3,EV_closestToChain,0,1,EV_leaderFar,0,3,EV_notClosestToChain,0,3 };
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
    const unsigned long int sup_data_prob_pos[6] = { 0,20,26,31,38,45 };
    const float             sup_data_prob[ 52 ] = { 2,1,1,2,1,1,2,1,1,2,1,1,1,1,1,1,1,1,1,1,0,1,1,0,1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1 };
    const unsigned long int sup_data_var_prob_pos[6] = { 0,12,14,16,19,22 };
    const unsigned char     sup_data_var_prob[ 25 ] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    float                   current_var_prob[0] = {  };
};

#endif