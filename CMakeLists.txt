cmake_minimum_required(VERSION 3.15)

project(CustomActuatorExample)

# === SOLUZIONE ARCHITETTURA (MANTIENI QUESTO!) ===
set(CMAKE_OSX_ARCHITECTURES "x86_64")
# =================================================

# Settings
set(TARGET exampleCustomActuator CACHE STRING "Nome del target eseguibile")
set(PLUGIN_TARGET SEA_Plugin_V2) # Nome della libreria

# C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find OpenSim
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# --- 1. CREAZIONE ESEGUIBILE (Quello che hai usato finora) ---
file(GLOB SOURCE_FILES *.h *.cpp)
add_executable(${TARGET} ${SOURCE_FILES})

# Link ed opzioni per l'eseguibile
target_link_libraries(${TARGET} ${OpenSim_LIBRARIES})
target_compile_options(${TARGET} PUBLIC "-fno-char8_t")

# --- 2. CREAZIONE PLUGIN (NUOVA PARTE PER LA GUI) ---
add_library(${PLUGIN_TARGET} SHARED SeriesElasticActuator.cpp Plugin_interface.cpp)

# Link ed opzioni per la libreria (identiche all'eseguibile)
target_link_libraries(${PLUGIN_TARGET} ${OpenSim_LIBRARIES})
target_compile_options(${PLUGIN_TARGET} PUBLIC "-fno-char8_t")

# --- Copia file ausiliari ---
# file(GLOB DATA_FILES *.vtp *.obj *.osim)
# foreach(dataFile ${DATA_FILES})
#     add_custom_command(
#         TARGET ${TARGET}
#         COMMAND ${CMAKE_COMMAND}
#         ARGS -E copy
#         ${dataFile}
#         ${CMAKE_BINARY_DIR})
# endforeach(dataFile)

# ==============================================================================
# 3. CREAZIONE DELL'ESEGUIBILE DI TEST
# ==============================================================================
# Questo target serve per eseguire il main() in Test_SEA.cpp.
# NOTA: Compiliamo SeriesElasticActuator.cpp anche qui per evitare problemi di linking su Mac.
add_executable(Test_SEA_App 
    
    Test_SEA_Acrobot.cpp
    SeriesElasticActuator.cpp
    CustomControl.cpp
)
#Test_SEA.cpp
# Link alle librerie di OpenSim
target_link_libraries(Test_SEA_App ${OpenSim_LIBRARIES})
target_compile_options(Test_SEA_App PUBLIC "-fno-char8_t")
