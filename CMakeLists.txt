cmake_minimum_required(VERSION 3.11...4.0)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqroboticspython)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    interfaces/cpp-interface-json11/include/
    interfaces/cpp-interface-json11/dropbox/json11/
)

add_subdirectory(pybind11)

pybind11_add_module(_dqrobotics

    src/dqrobotics_module.cpp

    src/DQ_py.cpp

    #Utils
    src/utils/DQ_LinearAlgebra_py.cpp
    src/utils/DQ_Geometry_py.cpp
    src/utils/DQ_Math_py.cpp

    #robot_modeling
    src/robot_modeling/DQ_CooperativeDualTaskSpace_py.cpp
    src/robot_modeling/DQ_Kinematics_py.cpp
    src/robot_modeling/DQ_SerialManipulator_py.cpp
    src/robot_modeling/DQ_SerialManipulatorDH_py.cpp
    src/robot_modeling/DQ_SerialManipulatorMDH_py.cpp
    src/robot_modeling/DQ_SerialManipulatorDenso_py.cpp
    src/robot_modeling/DQ_MobileBase_py.cpp
    src/robot_modeling/DQ_HolonomicBase_py.cpp
    src/robot_modeling/DQ_DifferentialDriveRobot_py.cpp
    src/robot_modeling/DQ_WholeBody_py.cpp
    src/robot_modeling/DQ_SerialWholeBody_py.cpp

    #robot_control
    src/robot_control/DQ_KinematicController_py.cpp
    src/robot_control/DQ_PseudoinverseController_py.cpp
    src/robot_control/DQ_NumericalFilteredPseudoInverseController_py.cpp
    src/robot_control/DQ_KinematicConstrainedController_py.cpp
    src/robot_control/DQ_QuadraticProgrammingController_py.cpp
    src/robot_control/DQ_ClassicQPController_py.cpp

    #solvers
    #DQ_QuadraticProgrammingSolver.h
    src/solvers/DQ_QuadraticProgrammingSolver_py.cpp

    #interfaces/json11
    interfaces/cpp-interface-json11/src/dqrobotics/interfaces/json11/DQ_JsonReader.cpp
    src/interfaces/json11/DQ_JsonReader_py.cpp
    interfaces/cpp-interface-json11/dropbox/json11/json11.cpp

)

# CMAKE was not passing BUILD_SHARED_LIBS nicely to the add_subdirectory.
# https://thatonegamedev.com/cpp/cmake/how-to-manage-dependencies-with-cmake/
set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(cpp)

target_link_libraries(_dqrobotics PRIVATE dqrobotics)