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
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds");
    
    /* Parse the configuration file */
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    /* Initialize the SCT controller */
    sct = new SCTProb();
    sct->add_callback(this, EV_a, &CExampleSCTProb::callback_a, NULL, NULL);
    sct->add_callback(this, EV_b, &CExampleSCTProb::callback_b, NULL, NULL);
    sct->add_callback(this, EV_c, NULL, &CExampleSCTProb::check_c, NULL);

    sct->print_current_state();
    std::cout << std::endl;
}

/****************************************/
/****************************************/

void CExampleSCTProb::ControlStep() {

    std::cout << "\n-------------" << std::endl;

    update_sensors();

    /* Run the generator player */
    sct->run_step();

    sct->print_current_state();
    std::cout << std::endl;

    ++time;
}

/****************************************/
/****************************************/

void CExampleSCTProb::update_sensors() {

}

/* Callback function for controllable events */
void CExampleSCTProb::callback_a(void* data) {
    std::cout << "ON" << std::endl;
    total_a++;
    m_pcLEDs->SetAllColors(CColor::RED);
}

void CExampleSCTProb::callback_b(void* data) {
    std::cout << "OFF" << std::endl;
    total_b++;
    m_pcLEDs->SetAllColors(CColor::BLACK);
}

/* Callback function for uncontrollable events */
unsigned char CExampleSCTProb::check_c(void* data) {
    return 1;
}

/* Callback function for updating variable probabilities */
// float CExampleSCTProb::check_prob_a(void* data) {
//     float prob = time/1000.;
//     // std::cout << "prob ON = " << prob << std::endl;
//     if(prob > 1){ prob = 1.; }
//     return prob;
// }

// float CExampleSCTProb::check_prob_b(void* data) {
//     float prob = 1 - time/1000.;
//     // std::cout << "prob OFF = " << prob << std::endl;
//     if(prob < 0){ prob = 0.; }
//     return prob;
// }

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
