// #include "SeriesElasticActuator.h"
// #include <OpenSim/OpenSim.h>


// using namespace OpenSim;
// using namespace SimTK;
// using namespace std;

// //==============================================================================
// // COSTRUTTORI
// //==============================================================================
// SeriesElasticActuator::SeriesElasticActuator() {
//     constructProperties();
//     // addStateVariable("motor_angle", Stage::Dynamics);
//     // addStateVariable("motor_speed", Stage::Dynamics);

//     cout <<"======== constructProperties(); ========"<<endl;
// }

// SeriesElasticActuator::SeriesElasticActuator(const std::string& name, double inertia, double damping, double k) {
//     constructProperties();
//     setName(name);
//     set_motor_inertia(inertia);
//     set_motor_damping(damping);
//     set_stiffness(k);

//     // OPZIONE 2
//     addStateVariable("motor_angle", Stage::Dynamics);
//     addStateVariable("motor_speed", Stage::Dynamics);
    
//     cout <<"======== constructProperties() 222 ========"<<endl;

// }

// //==============================================================================
// // COSTRUZIONE PROPRIETÀ
// //==============================================================================
// void SeriesElasticActuator::constructProperties() {
//     constructProperty_motor_inertia(0.1);  // Default: 0.1 kg*m^2
//     constructProperty_motor_damping(0.01); // Default: 0.01 Nms/rad
//     constructProperty_stiffness(1000.0);   // Default: 1000 N/m

//     // OPZIONE 3
//     // addStateVariable("motor_angle", Stage::Dynamics);
//     // addStateVariable("motor_speed", Stage::Dynamics);
// }

// void SeriesElasticActuator::extendAddToSystem(MultibodySystem& system) const {
//     Super::extendAddToSystem(system);

//     cout << "======== ENTRA IN extendAddToSystem ========" << endl;
//     // State variables definition for motor's dynamic
//     // addStateVariable("motor_angle", Stage::Dynamics);
//     // addStateVariable("motor_speed", Stage::Dynamics);
//     // SeriesElasticActuator* mutableThis = const_cast<SeriesElasticActuator*>(this);
    
//     // // Controllo per evitare duplicati se chiamato più volte
//     // if(getStateVariableNames().getSize() == 0) {
//     //     cout << " ======== ENTRA IN extendAddToSystem ========" << endl;
//     //     mutableThis->addStateVariable("motor_angle", Stage::Dynamics);
//     //     mutableThis->addStateVariable("motor_speed", Stage::Dynamics);
//     // }
// }
// void SeriesElasticActuator::extendConnectToModel(Model& model) {
//     Super::extendConnectToModel(model);

//     // Controlliamo che il socket sia connesso (Safety Check)
//     if(getSocket("coordinate").isConnected()) {
//         // Qui potremmo salvare puntatori raw se volessimo ottimizzare al massimo,
//         // ma per ora basta verificare la connessione.
//         // La logica è gestita dalle macro OpenSim.
//     }
// }

// // 2. Inizializzazione degli Stati
// void SeriesElasticActuator::extendInitStateFromProperties(SimTK::State& s) const {
//     Super::extendInitStateFromProperties(s);
    
//     // Inizializza esplicitamente a zero per evitare NaN al primo step
//     // Questo è fondamentale per evitare il crash a t=0
//     setStateVariableValue(s, "motor_angle", 0.0);
//     setStateVariableValue(s, "motor_speed", 0.0);
// }

// //==============================================================================
// // CALCOLO DERIVATE (DINAMICA MOTORE)
// //==============================================================================
// void SeriesElasticActuator::computeStateVariableDerivatives(const SimTK::State& s) const {
//     Super::computeStateVariableDerivatives(s);
//     cout << "[DEBUG] computeStateVariableDerivatives RUNNING inside component: " << getName() << std::endl;
//     // 1. Ottenere i parametri
//     double Jm = get_motor_inertia();
//     double Bm = get_motor_damping();
//     double K  = get_stiffness();
//     if (Jm < 1e-9) Jm = 1e-9;

//     // 2. Ottenere lo stato corrente del motore
//     double theta_m = getStateVariableValue(s, "motor_angle");
//     double omega_m = getStateVariableValue(s, "motor_speed");
//     cout << "theta_m" << theta_m << endl;

//     //-----------------------------------------------------------------
//     // const StateVariable& sv_angle = getStateVariable("motor_angle");
//     // const StateVariable& sv_speed = getStateVariable("motor_speed");

//     // Leggiamo i valori attuali
//     // double theta_m = sv_angle.getValue(s);
//     // double omega_m = sv_speed.getValue(s);
//     //-----------------------------------------------------------------

//     // 3. Ottenere lo stato della coordinata (giunto umano)
//     // Usiamo il socket per accedere all'oggetto Coordinate connesso
//     const Coordinate& coord = getConnectee<Coordinate>("coordinate");
//     double theta_joint = coord.getValue(s); 

//     // 4. Input di controllo (dal controller OpenSim)
//     // Nota: Se non c'è controller, getControl restituisce 0 o NaN se non gestito
//     double tau_input = 0.0;
//     if(!isCacheVariableValid(s, "control")) {
//          // Gestione di sicurezza se il controllo non è ancora stato calcolato
//          tau_input = 0.0; 
//     } else {
//          tau_input = getControls(s)[0];
//     }

//     if (std::abs(tau_input) > 0.001 || std::abs(omega_m) > 0.001) {
//         std::cout << "[SEA DEBUG] t=" << s.getTime() 
//                   << " | Input: " << tau_input 
//                   << " | MotAngle: " << theta_m 
//                   << " | MotSpeed: " << omega_m << std::endl;
//     }
//     // 5. Calcolo Coppia Elastica
//     double tau_spring = K * (theta_m - theta_joint);

//     // 6. Equazioni del moto del motore: J * acc = Tau_in - Tau_spring - Damping
//     double theta_m_dot = omega_m;
//     double omega_m_dot = (tau_input - tau_spring - (Bm * omega_m)) / Jm;

//     // 7. Impostare le derivate
//     setStateVariableDerivativeValue(s, "motor_angle", theta_m_dot); 
//     setStateVariableDerivativeValue(s, "motor_speed", omega_m_dot);
// }

// //==============================================================================
// // CALCOLO FORZA (INTERAZIONE COL MULTIBODY)
// //==============================================================================
// void SeriesElasticActuator::computeForce(const State& s, Vector_<SpatialVec>& bodyForces, Vector& generalizedForces) const {
    
//     // --- DEBUG DIAGNOSTICO (Da rimuovere dopo) ---
//     // Stampiamo i nomi delle variabili solo la prima volta
//     static bool debug_printed = false;
//     if (!debug_printed) {
//         std::cout << "\n[DIAGNOSTICA] Nomi Variabili Stato in " << getName() << ":" << std::endl;
//         Array<std::string> names = getStateVariableNames();
//         for(int i=0; i<names.getSize(); ++i) {
//             std::cout << " - '" << names[i] << "'" << std::endl;
//         }
//         debug_printed = true;
//     }
//     // ---------------------------------------------

//     double theta_m = 0.0;
//     try {
//         theta_m = getStateVariableValue(s, "motor_angle");
//     } catch (const std::exception&) {
//         if (s.getTime() > 0.0) {
//             std::cout << "[ERRORE CRITICO] computeForce fallisce al tempo " << s.getTime() << std::endl;
//         }
//         return; 
//     }

//     // 1. Parametri
//     double K = get_stiffness();
//     // double theta_m = getStateVariableValue(s, "motor_angle");
    
//     const Coordinate& coord = getConnectee<Coordinate>("coordinate");
//     double theta_joint = coord.getValue(s);

//     // 2. Calcolo forza trasmessa dalla molla all'articolazione
//     // Se theta_m > theta_joint, la molla "tira" il giunto in avanti (coppia positiva)
//     double tau_transmitted = K * (theta_m - theta_joint);

//     // 3. Applicazione tramite metodo helper di OpenSim::Actuator
//     // Questo applica la coppia generalizzata al grado di libertà corretto
//     applyGeneralizedForce(s, coord, tau_transmitted, generalizedForces);
// }

// //==============================================================================
// // CALCOLO POTENZA
// //==============================================================================
// double SeriesElasticActuator::getPower(const State& s) const {
//     double omega_m = getStateVariableValue(s, "motor_speed");
//     double tau_input = 0.0;
    
//     if(isCacheVariableValid(s, "control")) {
//          SimTK::Vector controls = getControls(s);
//          if(controls.size() > 0) tau_input = controls[0];
//     }
//     return tau_input * omega_m;
// }



/* -------------------------------------------------------------------------- *
 * SeriesElasticActuator.cpp - FIXED & CLEANED VERSION                        *
 * -------------------------------------------------------------------------- */
#include "SeriesElasticActuator.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==============================================================================
// COSTRUTTORI
//==============================================================================
SeriesElasticActuator::SeriesElasticActuator() {
    constructProperties();

    addStateVariable("motor_angle", Stage::Dynamics);
    addStateVariable("motor_speed", Stage::Dynamics);
}

SeriesElasticActuator::SeriesElasticActuator(const std::string& name, double inertia, double damping, double k) {
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
