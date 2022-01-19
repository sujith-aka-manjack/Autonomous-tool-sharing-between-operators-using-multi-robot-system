#include "robot_message.h"

/****************************************/
/****************************************/

Message::Message() {

}

/****************************************/
/****************************************/

Message::Message(CCI_RangeAndBearingSensor::SPacket packet) {

    //std::cout << packet.Data << std::endl;

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
    index += (2 - msg_num) * 4; // TEMP: Currently assuming only two teams

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
    index += (2 - msg_num) * 6; // TEMP: Currently assuming only two teams
    
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
    index += (2 - msg_num) * 1; // TEMP: Currently assuming only two teams

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
        if(relayMsg.time > 5000) {
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

        relayMsg.robot_num = packet.Data[index++];

        rmsg.push_back(relayMsg);
    }
    index += (2 - msg_num) * 8; // TEMP: Currently assuming only two teams

    /* Connections */
    while(packet.Data[index] != 255) {    // Check if data exists
        std::string robotID;
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        connections.push_back(robotID);
    }

}

/****************************************/
/****************************************/

Message::~Message() {

}

/****************************************/
/****************************************/

/* 
* Checks whether the Message is empty or not by checking the direction it was received from
*/
bool Message::Empty() {
    return direction.Length() == 0.0f;
}
