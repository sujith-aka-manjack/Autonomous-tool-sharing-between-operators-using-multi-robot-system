/* Include the controller definition */
#include "example_sctprob.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

CExampleSCTProb::CExampleSCTProb() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CExampleSCTProb::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"    );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing" );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing" );
    
    /* Parse the configuration file */
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    /* Initialize the SCT controller */
    sct = new SCTProb();
    sct->add_callback(this, EV_a, &CExampleSCTProb::callback_a, NULL, NULL);
    sct->add_callback(this, EV_b, &CExampleSCTProb::callback_b, NULL, NULL);
    sct->add_callback(this, EV_c, NULL, &CExampleSCTProb::check_c, NULL);
    sct->add_callback(this, EV_d, NULL, &CExampleSCTProb::check_d, NULL);

}

/****************************************/
/****************************************/

void CExampleSCTProb::ControlStep() {

    update_sensors();

    /* Run the generator player */
    sct->run_step();

}

/****************************************/
/****************************************/

void CExampleSCTProb::update_sensors() {
    c = 1;
    d = 0;
}

/* Callback function for controllable events */
void CExampleSCTProb::callback_a(void* data) {
    std::cout << "[" << this->GetId() << "] a" << std::endl;
    total_a++;
}

void CExampleSCTProb::callback_b(void* data) {
    std::cout << "[" << this->GetId() << "] b" << std::endl;
    total_b++;
}

/* Callback function for uncontrollable events */
unsigned char CExampleSCTProb::check_c(void* data) {
    if(c > 0) { return 1; }
    return 0;
}

unsigned char CExampleSCTProb::check_d(void* data) {
    if(d > 0) { return 1; }
    return 0;
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
REGISTER_CONTROLLER(CExampleSCTProb, "example_sctprob_controller")
