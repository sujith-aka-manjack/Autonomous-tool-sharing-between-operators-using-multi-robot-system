#include "robot_message.h"

/****************************************/
/****************************************/

Message::Message() {

}

/****************************************/
/****************************************/

Message::Message(CCI_RangeAndBearingSensor::SPacket packet) {

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
