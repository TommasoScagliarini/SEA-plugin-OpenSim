#include <OpenSim/OpenSim.h>
#include "SeriesElasticActuator.h" 

using namespace OpenSim;
using namespace SimTK;
using namespace std;

int main() {
    try {
        cout << "--- Avvio Test SeriesElasticActuator ---" << endl;

        // 1. Modello
        Model model;
        model.setName("SEA_Test");

        model.setGravity(Vec3(0, -9.80665, 0));
        // 2. Corpo (Pendolo semplice)
        // Ground& ground = model.updGround();
        // ground.attachGeometry(new Mesh("checkered_floor.vtp"));
        
        OpenSim::Body* body = new OpenSim::Body("pendulum", 1.0, Vec3(0.0, 0.5 / 2.0, 0.0), Inertia(0.1));
        Sphere sphere(0.05);
        body->attachGeometry(sphere.clone());
        Cylinder cyl(0.06/2, 0.5/2);

        Frame* cyl1Frame = new PhysicalOffsetFrame(*body, Transform(Vec3(0.0, 0.5 / 2.0, 0.0)));
        cyl1Frame->setName("Cyl1_frame");
        cyl1Frame->attachGeometry(cyl.clone());
        model.addComponent(cyl1Frame);

        model.addBody(body);

        // 3. Giunto
        PinJoint* joint = new PinJoint("joint", model.getGround(), Vec3(0,1,0), Vec3(0), *body, Vec3(0,0.5,0), Vec3(0));
        joint->setName("q_joint");
        model.addJoint(joint);

        // 4. Attuatore SEA
        SeriesElasticActuator* sea = new SeriesElasticActuator("my_sea", 0.1, 0.01, 100.0);
        sea->connectSocket_coordinate(joint->getCoordinate());
        model.addForce(sea);

        // 5. Controller (Input costante)
        PrescribedController* controller = new PrescribedController();
        controller->addActuator(*sea);
        controller->prescribeControlForActuator("my_sea", new Constant(5.0)); // 5 Nm al motore
        model.addController(controller);

        // 6. Simulazione
        model.setUseVisualizer(true);

        State& s = model.initSystem();
        cout << "=== model.initsystem eseguito correttamente ===" << endl;
        Manager manager(model);
        manager.initialize(s);
        cout << "===  manager.initialize(s) eseguito correttamente ===" << endl;

        manager.integrate(5.0);
        manager.setIntegratorAccuracy(1.0e-3);
        
        // 7. Output
        manager.getStateStorage().print("Test_SEA_Results.sto");
        cout << "--- Test Completato. File .sto generato ---" << endl;

        model.print("Test_SEA.osim");

    } catch (const std::exception& ex) {
        cout << "ERRORE: " << ex.what() << endl;
        return 1;
    }
    return 0;
}