/* Include the controller definition */
#include "leader.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

CLeader::CLeader() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CLeader::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"         );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing" );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing" );
    m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"       );
    
    /* Parse the configuration file */
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    // /* Initialize PID parameters */
    // pid_rotate = new PID(0.1, m_fWheelVelocity, -m_fWheelVelocity, 10, 0, 0);

    Reset();
}

/****************************************/
/****************************************/

void CLeader::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    msg = CByteArray(10, 255);
    m_pcRABAct->SetData(msg);
    msg_index = 0;

    /* Reset the incoming public events */
    pub_events.clear();
    pub_events[EV_b] = false;
}

/****************************************/
/****************************************/

void CLeader::ControlStep() {

    // std::cout << m_pcPosSens->GetReading().Position << std::endl;
    // std::cout << m_pcPosSens->GetReading().Orientation << std::endl;

    // CRadians cZAngle, cYAngle, cXAngle;
    

    // double turn_angle = pid_rotate->calculate(10, 11);  // TODO: Change input to radians

    // std::cout << turn_angle << std::endl;

    // m_pcWheels->SetLinearVelocity(turn_angle, -turn_angle);

    // get_messages();
    // update_sensors();

    /* Init new message */
    msg = CByteArray(10, 255);
    msg_index = 0;

    /* Send its orientation */
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    float n_angle = cZAngle.GetValue(); // Get angle in radians and store as float
    std::cout << "[L] " << n_angle << std::endl;

    unsigned char* value_ptr = reinterpret_cast<unsigned char*>(&n_angle);  // Convert float to 4 bytes
    for(int i = 0; i < sizeof(float); i++) {
        msg[msg_index++] = value_ptr[i];
        // std::cout << value_ptr[i] << std::endl;
    }

    /* Set message to send */
    m_pcRABAct->SetData(msg);

}

/****************************************/
/****************************************/

void CLeader::get_messages() {

    /* Reset all public event occurances */
    for(auto itr = pub_events.begin(); itr != pub_events.end(); ++itr) {
        itr->second = false;
    }

    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if(! tMsgs.empty()) {
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            size_t j = 0;
            while(tMsgs[i].Data[j] != 255) {    // Check all events in the message
                unsigned char event = tMsgs[i].Data[j];
                pub_events[event] = true;   // If a public event has occured, set it to true
                j++;
            }
        }
    }

    // for(auto itr = pub_events.begin(); itr != pub_events.end(); ++itr) {
    //     std::cout << "key = " << itr->first           // print key
    //               << ", val = " << itr->second << "\n";    // print value
    // }
}

void CLeader::update_sensors() {}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CLeader, "leader_controller")
