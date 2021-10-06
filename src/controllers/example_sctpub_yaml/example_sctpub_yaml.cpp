/* Include the controller definition */
#include "example_sctpub_yaml.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/****************************************/
/****************************************/

CExampleSCTPubYaml::CExampleSCTPubYaml() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CExampleSCTPubYaml::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"    );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing" );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing" );
    
    /* Parse the configuration file */
    GetNodeAttribute(t_node, "velocity", m_fWheelVelocity);
    GetNodeAttribute(t_node, "SCT", m_strSCTPath);

    /* Initialize the SCT controller */
    sct = new SCTPub(m_strSCTPath);
    sct->add_callback(this, sct->events["EV_a"], &CExampleSCTPubYaml::callback_a, NULL, NULL);
    sct->add_callback(this, sct->events["EV_b"], &CExampleSCTPubYaml::callback_b, NULL, NULL);
    sct->add_callback(this, sct->events["EV_c"], &CExampleSCTPubYaml::callback_c, NULL, NULL);
    sct->add_callback(this, sct->events["EV__b"], NULL, &CExampleSCTPubYaml::check__b, NULL);

    Reset();
}

/****************************************/
/****************************************/

CExampleSCTPubYaml::~CExampleSCTPubYaml() {
    delete sct;
}

/****************************************/
/****************************************/

void CExampleSCTPubYaml::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    msg = CByteArray(10, 255);
    m_pcRABAct->SetData(msg);
    msg_index = 0;

    /* Reset the incoming public events */
    pub_events.clear();
    pub_events[sct->events["EV_b"]] = false;
}

/****************************************/
/****************************************/

void CExampleSCTPubYaml::ControlStep() {

    get_messages();
    update_sensors();

    std::cout << sct->get_current_state_string() << std::endl;

    /* Init new message */
    msg = CByteArray(10, 255);
    msg_index = 0;

    /* Run the generator player */
    sct->run_step();

    /* Set message to send */
    m_pcRABAct->SetData(msg);
}

/****************************************/
/****************************************/

void CExampleSCTPubYaml::get_messages() {

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

void CExampleSCTPubYaml::update_sensors() {}

/* Callback function for controllable events */
void CExampleSCTPubYaml::callback_a(void* data) {
    std::cout << "[" << this->GetId() << "] a" << std::endl;
}

void CExampleSCTPubYaml::callback_b(void* data) {
    std::cout << "[" << this->GetId() << "] b" << std::endl;
    msg[msg_index++] = sct->events["EV_b"];
}

void CExampleSCTPubYaml::callback_c(void* data) {
    std::cout << "[" << this->GetId() << "] c" << std::endl;
}

/* Callback function for uncontrollable events */
unsigned char CExampleSCTPubYaml::check__b(void* data) {
    if(pub_events[sct->events["EV_b"]]) { return 1; }
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
REGISTER_CONTROLLER(CExampleSCTPubYaml, "example_sctpub_yaml_controller")
