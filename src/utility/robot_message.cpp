#include "robot_message.h"

/****************************************/
/****************************************/

UInt32 Message::teamCount;
UInt32 Message::messageByteSize;

/****************************************/
/****************************************/

void Message::SetTeamCount(size_t num_team) {
    teamCount = num_team;

    /*
    * size = fixed data + number of array messages + team count * (HopMsg + ConnectionMsg + TeamsNearby + RelayMsg) + Connections + End;
    */

    messageByteSize = (7 + 5) + 4 + teamCount * (4 + 6 + 1 + 10) + 60 + 1;
    // std::cout << int(messageByteSize) << std::endl;
}

/****************************************/
/****************************************/

Message::Message() {

}

/****************************************/
/****************************************/

Message::Message(CCI_RangeAndBearingSensor::SPacket packet) {

    size_t index = 0;

    /* Core */
    direction = CVector2(packet.Range, packet.HorizontalBearing);
    state = static_cast<RobotState>(packet.Data[index++]);
    ID = std::to_string(packet.Data[index++]); // Only stores number part of the id here
    teamID = packet.Data[index++];

    /* Leader Task Signal */
    leaderSignal = packet.Data[index++];

    /* Leader Team Switch Signal */
    if(packet.Data[index] != 255) {
        std::string switchID;
        switchID += (char)packet.Data[index++];            // First char of ID
        switchID += std::to_string(packet.Data[index++]);  // ID number
        robotToSwitch = switchID;
        teamToJoin = packet.Data[index++];
    } else
        index += 3;

    /* Hops */
    UInt8 msg_num = packet.Data[index++];

    if(msg_num == 255) // Safety check value
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {

        HopMsg hop;

        UInt8 tmpTeamID = packet.Data[index++];
        hop.count = packet.Data[index++];

        if(hop.count > 1) {
            std::string robotID;
            robotID += (char)packet.Data[index++];            // First char of ID
            robotID += std::to_string(packet.Data[index++]);  // ID number
            hop.ID = robotID;
        } else
            index += 2;
        
        hops[tmpTeamID] = hop;
    }
    index += (teamCount - msg_num) * 4;

    /* Connection Message */
    msg_num = packet.Data[index++];

    if(msg_num == 255)
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {

        ConnectionMsg conMsg;

        conMsg.type = (char)packet.Data[index++];

        std::string robotID;
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        conMsg.from = robotID;

        //std::cout << "FROM: " << conMsg.from << std::endl;

        robotID = "";
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        conMsg.to = robotID;
        
        //std::cout << "TO: " << conMsg.to << std::endl;
        
        conMsg.toTeam = packet.Data[index++]; 

        cmsg.push_back(conMsg);
    }
    index += (teamCount - msg_num) * 6;
    
    /* Shared Message */
    std::string robotID;
    if(packet.Data[index] != 255) {
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        shareToLeader = robotID;
    } else
        index += 2;
    
    if(packet.Data[index] != 255) {
        robotID = "";
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        shareToTeam = robotID;
    } else
        index += 2;

    shareDist = packet.Data[index++];

    /* Nearby Teams */
    msg_num = packet.Data[index++];

    if(msg_num == 255)
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {
        nearbyTeams.push_back(packet.Data[index++]);
    }
    index += (teamCount - msg_num) * 1;

    /* Relay Message */
    msg_num = packet.Data[index++];

    if(msg_num == 255)
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {

        RelayMsg relayMsg;

        relayMsg.type = (char)packet.Data[index++];

        std::string robotID;
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        relayMsg.from = robotID;

        // //std::cout << "FROM: " << relayMsg.from << std::endl;
        
        relayMsg.time = packet.Data[index++]*256 + packet.Data[index++]; 
        if(relayMsg.time > 65535) {
            std::cerr << "INVALID TIME RECEIVED from " << ID << std::endl;
            std::cerr << packet.Data << std::endl;
        }

        if(packet.Data[index] != 255) {
            robotID = "";
            robotID += (char)packet.Data[index++];            // First char of ID
            robotID += std::to_string(packet.Data[index++]);  // ID number
            relayMsg.firstFollower = robotID;
        } else
            index += 2;

        relayMsg.follower_num = packet.Data[index++];
        relayMsg.task_min_num = packet.Data[index++];
        relayMsg.robot_num = packet.Data[index++];

        rmsg.push_back(relayMsg);
    }
    index += (teamCount - msg_num) * 10;

    /* Connections */
    while(packet.Data[index] != 255) {    // Check if data exists
        std::string robotID;
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        connections.push_back(robotID);
    }

    // this->Print();

}

/****************************************/
/****************************************/

Message::~Message() {

}

/****************************************/
/****************************************/

CByteArray Message::GetCByteArray() {

    CByteArray arr = CByteArray(Message::messageByteSize, 255);
    size_t index = 0;

    /* Sender State */
    arr[index++] = static_cast<UInt8>(state);

    /* Sender ID */
    arr[index++] = stoi(ID.substr(1));

    /* Sender TeamID */
    arr[index++] = teamID;

    /* Leader Signal */
    arr[index++] = leaderSignal;

    /* Team Switch */
    if( !robotToSwitch.empty() ) {
        arr[index++] = robotToSwitch[0];
        arr[index++] = stoi(robotToSwitch.substr(1));
        arr[index++] = teamToJoin;
    } else
        index += 3;

    /* Hop count */
    arr[index++] = hops.size(); // Set the number of HopMsg
    for(const auto& it : hops) {

        arr[index++] = it.first;                     // Team ID
        arr[index++] = it.second.count;              // Count

        if( it.second.ID.empty() )
            index += 2; // Skip
        else {
            arr[index++] = it.second.ID[0];              // ID
            arr[index++] = stoi(it.second.ID.substr(1)); // ID
        }
    }
    // Skip if not all bytes are used
    index += (teamCount - hops.size()) * 4;

    /* Connection Message */
    arr[index++] = cmsg.size(); // Set the number of ConnectionMsg
    for(const auto& conMsg : cmsg) {
        arr[index++] = (UInt8)conMsg.type;
        arr[index++] = conMsg.from[0];
        arr[index++] = stoi(conMsg.from.substr(1));
        arr[index++] = conMsg.to[0];
        arr[index++] = stoi(conMsg.to.substr(1));
        arr[index++] = conMsg.toTeam;
    }
    // Skip if not all bytes are used
    index += (teamCount - cmsg.size()) * 6;

    /* Shared Message */
    if( !shareToLeader.empty() ) {
        arr[index++] = shareToLeader[0];
        arr[index++] = stoi(shareToLeader.substr(1));
    } else
        index += 2;

    if( !shareToTeam.empty() ) {
        arr[index++] = shareToTeam[0];
        arr[index++] = stoi(shareToTeam.substr(1));
    } else
        index += 2;

    arr[index++] = shareDist;

    /* Teams Nearby */
    arr[index++] = nearbyTeams.size(); // Set the number of nearby teams
    for(const auto& id : nearbyTeams) {
        arr[index++] = id;
    }
    // Skip if not all bytes are used
    index += (teamCount - nearbyTeams.size()) * 1;

    /* Relay Message */
    arr[index++] = rmsg.size(); // Set the number of RelayMsg
    for(const auto& relayMsg : rmsg) {
        arr[index++] = (UInt8)relayMsg.type;
        arr[index++] = relayMsg.from[0];
        arr[index++] = stoi(relayMsg.from.substr(1));
        arr[index++] = (UInt8)(relayMsg.time / 256.0);
        arr[index++] = (UInt8)(relayMsg.time % 256);

        if( !relayMsg.firstFollower.empty() ) {
            arr[index++] = relayMsg.firstFollower[0];
            arr[index++] = stoi(relayMsg.firstFollower.substr(1));
        } else
            index += 2;

        arr[index++] = relayMsg.follower_num;
        arr[index++] = relayMsg.task_min_num;
        arr[index++] = relayMsg.robot_num;
    }
    // Skip if not all bytes are used
    index += (teamCount - rmsg.size()) * 10;

    /* Connections */
    for(size_t i = 0; i < connections.size(); i++) {
    
        //std::cout << allMsgs[i].ID << ", ";

        arr[index++] = connections[i][0];    // First character of ID
        arr[index++] = stoi(connections[i].substr(1));    // ID number

        if(i >= 29){
            std::cerr << "[" << ID << "] max connections reached" << std::endl;
            break;
        }
    }

    return arr;
}

/****************************************/
/****************************************/

/* 
* Checks whether the Message is empty or not by checking the direction it was received from
*/
bool Message::Empty() {
    return direction.Length() == 0.0f;
}

/****************************************/
/****************************************/

void Message::Print() {

    std::cout << "\n##########" << std::endl;

    switch(state) {
        case RobotState::LEADER:
            std::cout << "state: LEADER" << std::endl;
            break;
        case RobotState::FOLLOWER:
            std::cout << "state: FOLLOWER" << std::endl;
            break;
        case RobotState::CONNECTOR:
            std::cout << "state: CONNECTOR" << std::endl;
            break;
        case RobotState::TRAVELER:
            std::cout << "state: TRAVELER" << std::endl;
            break;
        default:
            /* The message is not initialised */
            if(int(state) != 255) {
                std::cerr << "Unknown state " << int(state) << " in " << ID << std::endl;
            }
            break;
    }

    std::cout << "ID: " << ID << std::endl;

    std::cout << "teamID: " << teamID << std::endl;

    std::cout << "leaderSignal: " << leaderSignal << std::endl;

    std::cout << "robotToSwitch: " << robotToSwitch << std::endl;

    std::cout << "teamToJoin: " << teamToJoin << std::endl;

    std::cout << "hops:" << std::endl;
    for(const auto& hop : hops) {
        std::cout << "--- team: " << hop.first
                  << ", count: " << hop.second.count
                  << ", ID: " << hop.second.ID
                  << std::endl;
    }

    std::cout << "cmsg:" << std::endl;
    for(const auto& conMsg : cmsg) {
        std::cout << "--- type: " << conMsg.type
                  << ", from: " << conMsg.from
                  << ", to: " << conMsg.to
                  << ", toTeam: " << conMsg.toTeam
                  << std::endl;
    }

    std::cout << "shareToLeader: " << shareToLeader << std::endl;

    std::cout << "shareToTeam: " << shareToTeam << std::endl;

    std::cout << "shareDist: " << shareDist << std::endl;

    std::cout << "nearbyTeams:" << std::endl;
    for(const auto& team : nearbyTeams) {
        std::cout << "--- team: " << team << std::endl;
    }

    std::cout << "rmsg:" << std::endl;
    for(const auto& relayMsg : rmsg) {
        std::cout << "--- type: " << relayMsg.type
                  << ", from: " << relayMsg.from
                  << ", time: " << relayMsg.time
                  << ", firstFollower: " << relayMsg.firstFollower
                  << ", follower_num: " << relayMsg.follower_num
                  << ", task_min_num: " << relayMsg.task_min_num
                  << ", robot_num: " << relayMsg.robot_num
                  << std::endl;
    }

    std::cout << "connections: ";
    for(const auto& connection : connections) {
        std::cout << connection << ",";
    }
    std::cout << std::endl;


}
