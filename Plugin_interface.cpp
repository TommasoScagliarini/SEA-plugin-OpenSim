/* -------------------------------------------------------------------------- *
 * Plugin_interface.cpp                                  *
 * -------------------------------------------------------------------------- */
// Includiamo gli header di TUTTI gli oggetti che vogliamo registrare
#include "SeriesElasticActuator.h"

// Header standard di OpenSim e C++
#include <OpenSim/OpenSim.h>
#include <cstdio> 

using namespace OpenSim;
using namespace std;

// =========================================================================
// LOGICA DI AUTO-REGISTRAZIONE (Spostata qui)
// =========================================================================

// Questa funzione è il "Costruttore della Libreria".
// Il sistema operativo la esegue AUTOMATICAMENTE appena carica il file .dylib.
// Non devi chiamarla tu manualmente!
__attribute__((constructor))
void DllMain() {
    
    printf("\n\n****************************************************************\n");
    printf("[DEBUG] >>> SYSTEM ALERT: IL PLUGIN E' STATO CARICATO IN MEMORIA.\n");
    printf("[DEBUG] >>> INIZIO REGISTRAZIONE OGGETTI...\n");
    fflush(stdout);

    try {
        Object::registerType(SeriesElasticActuator());
        printf("[DEBUG] >>> SEA: registered.\n");
        
        printf("****************************************************************\n\n");
        fflush(stdout);
    } 
    catch (const std::exception& e) {
        printf("[DEBUG] >>> ERROR DURING REGISTRATION: %s \n", e.what());
        fflush(stdout);
    }
}

// Funzione dummy per compatibilità con vecchie versioni di OpenSim
// (OpenSim cerca questo nome specifico, anche se noi abbiamo già fatto tutto sopra)
// extern "C" void RegisterTypes_OsimCustomActuatorPlugin() {
//     // Non fa nulla, l'auto-registrazione è già avvenuta.
// }