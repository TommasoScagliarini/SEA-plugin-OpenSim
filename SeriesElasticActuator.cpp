
#include "SeriesElasticActuator.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==============================================================================
// COSTRUTTORI
//==============================================================================
SeriesElasticActuator::SeriesElasticActuator() {
    setAuthors("Tommaso Scagliarini");
    setReferences("Series Elastic Actuator plugin for OpenSim");
    constructProperties();

    addStateVariable("motor_angle", Stage::Dynamics);
    addStateVariable("motor_speed", Stage::Dynamics);
}

SeriesElasticActuator::SeriesElasticActuator(const std::string& name, double inertia, double damping, double k) {
    setAuthors("Tommaso Scagliarini");
    setReferences("Series Elastic Actuator plugin for OpenSim");
    constructProperties();
    
    setName(name);
    set_motor_inertia(inertia);
    set_motor_damping(damping);
    set_stiffness(k);

    
    // addStateVariable("motor_angle", Stage::Dynamics);
    // addStateVariable("motor_speed", Stage::Dynamics);
}

//==============================================================================
// COSTRUZIONE PROPRIETÀ
//==============================================================================
void SeriesElasticActuator::constructProperties() {
    constructProperty_motor_inertia(0.1);  
    constructProperty_motor_damping(0.01); 
    constructProperty_stiffness(1000.0);   
}

//==============================================================================
// 1. REGISTRAZIONE NEL SISTEMA 
//==============================================================================
void SeriesElasticActuator::extendAddToSystem(MultibodySystem& system) const {

    SeriesElasticActuator* mutableThis = const_cast<SeriesElasticActuator*>(this);
    if(mutableThis->getNumStateVariables() == 0) {
        
        mutableThis->addStateVariable("motor_angle", Stage::Dynamics);
        mutableThis->addStateVariable("motor_speed", Stage::Dynamics);
    }

    Super::extendAddToSystem(system);
}

//==============================================================================
// 2. CONNESSIONE MODELLO
//==============================================================================
void SeriesElasticActuator::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    // Safety check socket connection
    if(!getSocket("coordinate").isConnected()) {
        std::cerr << "[SEA WARNING] Socket 'coordinate' is not connected!" << std::endl;
    }
}

//==============================================================================
// 3. INIZIALIZZAZIONE DEGLI STATI
//==============================================================================
void SeriesElasticActuator::extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    
    // Initialization
    setStateVariableValue(s, "motor_angle", 0.0);
    setStateVariableValue(s, "motor_speed", 0.0);
}

//==============================================================================
// 4. CALCOLO DERIVATE (DINAMICA MOTORE)
//==============================================================================
void SeriesElasticActuator::computeStateVariableDerivatives(const SimTK::State& s) const {
    
    // Get Motor's parameters
    double Jm = get_motor_inertia();
    double Bm = get_motor_damping();
    double K  = get_stiffness();
    if (Jm < 1e-9) Jm = 1e-9;

    double theta_m = 0.0; 
    double omega_m = 0.0;
    theta_m = getStateVariableValue(s, "motor_angle");
    omega_m = getStateVariableValue(s, "motor_speed");
    
    // Get coordinate's state
    const Coordinate& coord = getConnectee<Coordinate>("coordinate");
    double theta_joint = coord.getValue(s); 


    // Control input
    double tau_input = 0.0;

    SimTK::Vector controls = getControls(s);
    if (controls.size() > 0) {
        tau_input = controls[0];
     }

    // Debug: stampa solo ogni tanto o se c'è input significativo
    if (std::abs(tau_input) > 0.001) {
       printf("[SEA] t=%.3f | tau_input=%.2f | motor agle=%.2f | joint angle=%.2f\n", s.getTime(), tau_input, theta_m, theta_joint);
    }

    // 5. Elastic torque computation
    double tau_spring = K * (theta_m - theta_joint);

    // 6. Motion equations
    double theta_m_dot = omega_m;
    double omega_m_dot = (tau_input - tau_spring - (Bm * omega_m)) / Jm;

    // 7. Set the derivatives
    setStateVariableDerivativeValue(s, "motor_angle", theta_m_dot); 
    setStateVariableDerivativeValue(s, "motor_speed", omega_m_dot);
    //Super::computeStateVariableDerivatives(s);

}

//==============================================================================
// 5. CALCOLO FORZA (INTERAZIONE COL MULTIBODY)
//==============================================================================
void SeriesElasticActuator::computeForce(const State& s, Vector_<SpatialVec>& bodyForces, Vector& generalizedForces) const {
    
    double theta_m = 0.0;
    theta_m = getStateVariableValue(s, "motor_angle");

    double K = get_stiffness();
    const Coordinate& coord = getConnectee<Coordinate>("coordinate");
    double theta_joint = coord.getValue(s);

    double tau_transmitted = K * (theta_m - theta_joint);

    applyGeneralizedForce(s, coord, tau_transmitted, generalizedForces);
}

//==============================================================================
// 6. POTENZA
//==============================================================================
double SeriesElasticActuator::getPower(const State& s) const {
    double omega_m = 0.0;
    try {
        omega_m = getStateVariableValue(s, "motor_speed");
    } catch(...) { return 0.0; }

    double tau_input = 0.0;
    if(isCacheVariableValid(s, "control")) {
         SimTK::Vector controls = getControls(s);
         if(controls.size() > 0) tau_input = controls[0];
    }
    return tau_input * omega_m;
}
