// #include <OpenSim/OpenSim.h>
// #include "CustomControl.h"

// using namespace OpenSim;
// using namespace SimTK;
// using namespace std;


// double desiredModelZPosition( double t ) {
//     return 0.15 * sin( Pi * t );
// }

// double desiredModelZVelocity( double t ) {
//     return 0.15 * Pi * cos( Pi * t );
// }

// double desiredModelZAcceleration( double t ) {
//     return -0.15 * Pi * Pi * sin( Pi * t );
// }


// class CustomController : public Controller {
// OpenSim_DECLARE_CONCRETE_OBJECT(CustomController, Controller);
// OpenSim_DECLARE_SOCKET(coordinate, Coordinate, "Coordinata da controllare");

// public:
// 	CustomController(double aKp, double aKv) : Controller(), kp( aKp ), kv( aKv ) 
//     {
//         constructSocket_coordinate();
//     }

//     void computeControls(const SimTK::State& s, SimTK::Vector &controls) const override
//     {
//         double t = s.getTime();
//         //double blockMass = getModel().getBodySet().get( "block" ).getMass();

//         // auto leftMuscle = dynamic_cast<const Muscle*>  ( &getActuatorSet().get(0) );
//         // auto rightMuscle = dynamic_cast<const Muscle*> ( &getActuatorSet().get(1) );

//         double pos_des = desiredModelZPosition(t);
//         double vel_des = desiredModelZVelocity(t);
//         double acc_des = desiredModelZAcceleration(t);

//         Coordinate& coord = getConnectee<Coordinate>("coordinate");

//         double z  = coord.getValue(s);
//         double zv = coord.getSpeedValue(s);

//         double pErrTerm = kp * ( pos_des  - z  );
//         double vErrTerm = kv * ( vel_des - zv );

//         // double desAcc = zdesa + pErrTerm + vErrTerm;
//         // double desFrc = desAcc * blockMass;

//         double u = 0.0;
        
//         u = kp*(pos_des-z) + kv*(vel_des - zv);

//         SimTK::Vector actuatorControl(1, u);
//         const Actuator& act = getActuatorSet();
//         act.addInControls(actuatorControl, controls);

//         // double FoptL = leftMuscle->getMaxIsometricForce();
//         // double FoptR = rightMuscle->getMaxIsometricForce();

//         // double leftControl = leftMuscle->getMinControl(),
//         //     rightControl = rightMuscle->getMinControl();
//         // if( desFrc < 0 ) {
//         //     leftControl = std::abs( desFrc ) / FoptL;
//         // }
//         // else if( desFrc > 0 ) {
//         //     rightControl = std::abs( desFrc ) / FoptR;
//         // }

//         // if( leftControl > leftMuscle->getMaxControl())
//         //     leftControl = leftMuscle->getMaxControl();
//         // if( rightControl > rightMuscle->getMaxControl())
//         //     rightControl = rightMuscle->getMaxControl();

//         // Vector muscleControl(1, leftControl);
//         // leftMuscle->addInControls(muscleControl, controls);
//         // muscleControl[0] = rightControl;
//         // rightMuscle->addInControls(muscleControl, controls);
//     }

// private:
//     double kp;
//     double kv;
// };

/* CustomControl.cpp */
#include "CustomControl.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

// Funzioni ausiliarie
double desiredModelZPosition( double t ) {
    return 1* sin( 0.5 * t ); //pene
}

double desiredModelZVelocity( double t ) {
    return 1* 0.5 * cos( 0.5 * t );
}

// ============================================================================
// IMPLEMENTAZIONE
// ============================================================================

CustomController::CustomController() : Controller() {
    constructProperties();
    // RIMOSSO: constructSocket_coordinate(); NON ESISTE
}

CustomController::CustomController(double aKp, double aKv) : Controller() 
{
    constructProperties();
    // RIMOSSO: constructSocket_coordinate(); NON ESISTE
    set_kp(aKp);
    set_kv(aKv);
}

void CustomController::constructProperties() {
    constructProperty_kp(100.0);
    constructProperty_kv(20.0);
}

void CustomController::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
{
    double t = s.getTime();
    double kp = get_kp();
    double kv = get_kv();

    double pos_des = desiredModelZPosition(t);
    double vel_des = desiredModelZVelocity(t);

    const Coordinate& coord = getConnectee<Coordinate>("coordinate");

    double z  = coord.getValue(s);
    double zv = coord.getSpeedValue(s);

    double u = kp * (pos_des - z) + kv * (vel_des - zv);

    // CORREZIONE TIPO: const Set<const Actuator>&
    // OpenSim restituisce un set di attuatori "const" per sicurezza
    const Set<const Actuator>& actuatorSet = getActuatorSet();
    
    if(actuatorSet.getSize() > 0) {
        // Prendo il primo attuatore e aggiungo il controllo
        const Actuator& act = actuatorSet.get(0);
        SimTK::Vector actuatorControl(1, u);
        act.addInControls(actuatorControl, controls);
    }
}