/* Include the controller definition */
#include "follower.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

CFollower::CFollower() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CFollower::Init(TConfigurationNode& t_node) {

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

void CFollower::Reset() {

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

void CFollower::ControlStep() {

    // std::cout << m_pcPosSens->GetReading().Position << std::endl;
    // std::cout << m_pcPosSens->GetReading().Orientation << std::endl;    

    // double turn_angle = pid_rotate->calculate(10, 11);  // TODO: Change input to radians

    // std::cout << turn_angle << std::endl;


    // get_messages();
    // update_sensors();

    // std::cout << m_pcPosSens->GetReading().Orientation << std::endl;

    // CRadians cZAngle, cYAngle, cXAngle;
    // m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    // std::cout << cZAngle << std::endl;

    // m_pcWheels->SetLinearVelocity(1, -1);

    /* Init new message */
    msg = CByteArray(10, 255);
    msg_index = 0;

    get_messages();

    /* Calculate flocking vectors */
    CVector2 vec_align = alignment();
    print_name();
    std::cout << "align: " << vec_align << std::endl;

    CVector2 vec_attract = cohesion();
    print_name();
    std::cout << "att: " << vec_attract << std::endl;

    CVector2 vec_repel = repulsion();
    print_name();
    std::cout << "repel: " << vec_repel << std::endl;

    /* Combine vectors */
    CVector2 vec_sum = vec_align + vec_attract + vec_repel;
    print_name();
    std::cout << "sum: " << vec_sum << std::endl;

    flock(vec_sum);

    // /* Run the generator player */
    // sct->run_step();

    /* Set message to send */
    m_pcRABAct->SetData(msg);

}

/****************************************/
/****************************************/

void CFollower::get_messages() {

    /* Reset all public event occurances */
    for(auto itr = pub_events.begin(); itr != pub_events.end(); ++itr) {
        itr->second = false;
    }

    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if(! tMsgs.empty()) {

        for(size_t i = 0; i < tMsgs.size(); ++i) {

            size_t index = 0;

            /* Get robot state */
            size_t r_state = tMsgs[i].Data[index++];

            /* Get robot team */
            size_t r_team = tMsgs[i].Data[index++];

            /* If Leader, get orientation */
            if(r_state == 0) {
                
                std::vector<char> val(4);
                for(size_t j = 0; j < 4; ++j) {
                    // std::cout << tMsgs[0].Data[i] << std::endl;
                    val[j] = tMsgs[i].Data[index++];
                }

                leader_n = *reinterpret_cast<float*>(&val[0]);
                std::cout << "[F1] " << leader_n << std::endl;

                leader_range = tMsgs[i].Range;
                leader_bearing = tMsgs[i].HorizontalBearing;
            }
        }
    }

    // for(auto itr = pub_events.begin(); itr != pub_events.end(); ++itr) {
    //     std::cout << "key = " << itr->first           // print key
    //               << ", val = " << itr->second << "\n";    // print value
    // }
}

void CFollower::update_sensors() {}

CVector2 CFollower::alignment() {
    return CVector2(1, CRadians(leader_n));
}

CVector2 CFollower::cohesion() {
    CVector2 vec;

    if(leader_range > thres_attract) {
        vec.FromPolarCoordinates(1, leader_bearing);
    }

    return vec;
}

CVector2 CFollower::repulsion() {
    CVector2 vec;

    if(leader_range < thres_repel) {
        vec.FromPolarCoordinates(1, -leader_bearing);
    }

    return vec;
}

void CFollower::flock(CVector2 vec) {
    print_name();
    std::cout << vec.Angle().GetAbsoluteValue() << std::endl;
    print_name();
    std::cout << vec.Length() << std::endl;

    if (vec.Length() >= 1) {
        if (vec.Angle().GetAbsoluteValue() > 3.14/8) {
            print_name();
            std::cout << "rotate" << std::endl;
        } else {
            print_name();
            std::cout << "stop, small angle" << std::endl;
        }
    } else {
        print_name();
        std::cout << "stop, short vec" << std::endl;
    }
    
}

void CFollower::print_name() {
    std::cout << "[" << this->GetId() << "] ";
}

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
REGISTER_CONTROLLER(CFollower, "follower_controller")
