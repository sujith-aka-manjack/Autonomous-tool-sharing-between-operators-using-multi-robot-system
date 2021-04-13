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
    sct->add_callback(this, EV_c, &CExampleSCTProb::callback_c, NULL, NULL);
    sct->add_callback(this, EV_d, &CExampleSCTProb::callback_d, NULL, NULL);

    sct->add_variable_prob(this, 0, 1, EV_c, &CExampleSCTProb::check_prob_c, NULL);
    sct->add_variable_prob(this, 0, 1, EV_d, &CExampleSCTProb::check_prob_d, NULL);

}

/****************************************/
/****************************************/

void CExampleSCTProb::ControlStep() {

    update_sensors();

    /* Run the generator player */
    sct->run_step();

    ++time;
}

/****************************************/
/****************************************/

void CExampleSCTProb::update_sensors() {

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

void CExampleSCTProb::callback_c(void* data) {
    std::cout << "[" << this->GetId() << "] c" << std::endl;
    total_c++;
}

void CExampleSCTProb::callback_d(void* data) {
    std::cout << "[" << this->GetId() << "] d" << std::endl;
    total_d++;
}

/* Callback function for uncontrollable events */


/* Callback function for updating variable probabilities */
float CExampleSCTProb::check_prob_c(void* data) {
    float prob = time/1000.;
    if(prob > 1){ prob = 1.; }
    return prob;
}

float CExampleSCTProb::check_prob_d(void* data) {
    float prob = 1 - time/1000.;
    if(prob < 0){ prob = 0.; }
    return prob;
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
