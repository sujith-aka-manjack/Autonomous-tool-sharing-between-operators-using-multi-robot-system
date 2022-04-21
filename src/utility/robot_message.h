/*
* AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
* 
* Define the message structure used to communicate between the robots.
*/

#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

/*
 * Include some necessary headers.
 */

/* Definition of the CVector2 datatype */
#include <argos3/core/utility/math/vector2.h>

/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <map>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/* List of states */
enum class RobotState {
    LEADER = 0,
    FOLLOWER,
    CONNECTOR,
    TRAVELER
};

/* Structure to store the connection to the leader/team */
struct HopMsg {
    UInt8 count;
    std::string ID; // Robot with lower hop value (only used by connectors)
};

/*
* Structure to store request/approval messages for extending the chain.
* 
*   - R (Request) : Follower sends to leader or connector 
*   - A (Accept)  : Leader or connector sends to follower
* 
*       Structure: Type [1], sender ID [2], recipient ID [2], recipient team ID [1] (for Connector -> Follower accepts) 
*/
struct ConnectionMsg {
    char type = 'N'; // R or A or N (none)
    std::string from;
    std::string to;
    UInt8 toTeam;
};

/* Message sent by a leader to other leaders */
struct RelayMsg {
    char type = 'H'; // H (heart-beat) or R (request-robot) or A (acknowledge)
    std::string from;
    UInt16 time;
    std::string firstFollower; // First follower that received this message from a non-team robot
    UInt8 follower_num; // Used in a heart-beat message. The number of followers the leader has.
    UInt8 task_min_num = 0; // Used in a heart-beat message. The minimum number of robots required to perform the task that the leader is on.
    UInt8 robot_num = 0; // Used in request-robot and acknowledge message. The number of robots the leader is requesting or sending.
};

// static const UInt32 MESSAGE_BYTE_SIZE = 119;

/* 
* Structure to store incoming data received from other robots 
* 
* The raw messages are assumed to arrive in the following data structure:
* 
* |  (1)   |  (2)   |   (3)   |  (4)   |  (5)-(7)  | (8)-(16)  |  (17)-(29) | (30)-(34) | (35)-(37) | (38)-(58) |      (59)-(118)       | (119) |
* -----------------------------------------------------------------------------------------------------------------------------------------------
* | Sender | Sender | Sender  | Leader |   Team    | Hop count | Connection |  Shared   |   Teams   |   Relay   |      Connections      |  End  |
* | State  |   ID   | Team ID | Signal |  Switch   |           |  Message   |  Message  |   Nearby  |  Message  | (2 bytes for ID x 30) | (255) |
* 
* 
* - (4) Leader Signal
*   - Leader    : task signal [1]
* 
* - (5)-(7) Team Switch Signal
*   - Leader informs a follower to join another team
*       - robotID [2]
*       - teamID [1]
* 
* - (8)-(16) Hop count
*   Prefix with number of messages (max 2) [1]
*   - HopMsg [4] (teamID [1], count [1], ID [2])
* 
* - (17)-(29) Connection Message
*   Prefix with number of messages (max 2) [1]
*   - ConnectionMsg [6]
* 
*       - Exchanging ConnectionMsg within a team:
*           - If message destination is to leader, relay upstream
*           - If message sender is the leader, relay downstream
*
*       - Exchanging ConnectionMsg between follower and connector:
*           - Follower will send up to one request message (R)
*           - Connector will send up to two approval messages (A)
* 
* - (30)-(34) Shared Message
* 
*       - Share information about the closest connector to the team
*           - shareToLeader: Upstream (Follower to Leader)
*           - shareToTeam  : Downstream (Leader to Follower)
*       - Share information about the shortest distance to the other team (only when no connector is detected)
*           - shareDist    : Upstream (Follower to Leader) 
* 
* - (35)-(37) Teams Nearby
*   Prefix with number of teams nearby (max 2) [1]
*   - teamID [1]
* 
*       - Used by connectors to determine whether other connectors can switch to a follower
* 
* - (38)-(58) Relay Message
*   Prefix with number of messages (max 2) [1]
*   - RelayMsg [10] (Type [1], Leader ID [2], time sent [2], first follower [2], follower_num [1], task_min_num [1]. robot_num [1])
* 
*       - Message sent by a leader to other leaders
* 
*/
class Message {

    public:

        /* Static members */
        static UInt32 teamCount;
        static UInt32 messageByteSize;

        static void SetTeamCount(size_t num_team);

        /* Class constructor */
        Message();

        Message(CCI_RangeAndBearingSensor::SPacket packet);

        virtual ~Message();

        virtual CByteArray GetCByteArray();

        virtual bool Empty();

        virtual void Print();

    public:

        /* Core */
        CVector2 direction;
        RobotState state;
        std::string ID;
        UInt8 teamID;

        /* Leader Signal */
        UInt8 leaderSignal;

        /* Team Switch */
        std::string robotToSwitch = "";
        UInt8 teamToJoin = 0;

        /* Hop Count */
        std::map<UInt8, HopMsg> hops; // Key is teamID

        /* Connection Message*/
        std::vector<ConnectionMsg> cmsg;

        /* Shared Message */
        std::string shareToLeader = "";
        std::string shareToTeam = "";
        UInt8 shareDist = 255;

        /* Teams Nearby */
        std::vector<UInt8> nearbyTeams;

        /* Relay Message */
        std::vector<RelayMsg> rmsg;

        /* Connections */
        std::vector<std::string> connections;

};

#endif