/* Include the controller definition */
#include "example_sct.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

CExampleSCT::CExampleSCT() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CExampleSCT::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"    );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing" );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing" );
    
    /* Parse the configuration file */
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    /* Initialize the SCT controller */
    sct = new SCT();
    sct->add_callback(this, EV_a, &CExampleSCT::callback_a, NULL, NULL);
    sct->add_callback(this, EV_b, &CExampleSCT::callback_b, NULL, NULL);
    sct->add_callback(this, EV_c, NULL, &CExampleSCT::check_c, NULL);
    sct->add_callback(this, EV_d, NULL, &CExampleSCT::check_d, NULL);

    Reset();
}

/****************************************/
/****************************************/

CExampleSCT::~CExampleSCT() {
    std::cout << "total_a: " << total_a << ", total_b: " << total_b << std::endl;
    delete sct;
}

/****************************************/
/****************************************/

void CExampleSCT::ControlStep() {

    update_sensors();

    std::cout << sct->get_current_state_string() << std::endl;

    /* Run the generator player */
    sct->run_step();
}

/****************************************/
/****************************************/

void CExampleSCT::update_sensors() {
    c = 1;
    d = 0;
}

/* Callback function for controllable events */

void CExampleSCT::callback_a(void* data) {
    std::cout << "[" << this->GetId() << "] a" << std::endl;
    total_a++;
}

void CExampleSCT::callback_b(void* data) {
    std::cout << "[" << this->GetId() << "] b" << std::endl;
    total_b++;
}

/* Callback function for uncontrollable events */

unsigned char CExampleSCT::check_c(void* data) {
    if(c > 0) { return 1; }
    return 0;
}

unsigned char CExampleSCT::check_d(void* data) {
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
REGISTER_CONTROLLER(CExampleSCT, "example_sct_controller")
