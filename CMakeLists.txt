cmake_minimum_required(VERSION 3.1)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqroboticspython)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#COPPELIASIM INTERFACE
find_package(cppzmq REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(Boost REQUIRED)
add_definitions(-DSIM_REMOTEAPICLIENT_OBJECTS)

#VREP INTERFACE
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)

if(APPLE)
    message("Building not supported on this platform.")
    include_directories(
        /usr/local/include/
        /usr/local/include/eigen3
        /opt/homebrew/include
        /opt/homebrew/include/eigen3
    )
endif()

if(WIN32)
    message("Building not supported on this platform.")

    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif()


include_directories(
    /usr/include/eigen3/

    cpp/include/

    interfaces/cpp-interface-vrep/include/
    interfaces/cpp-interface-vrep/coppeliarobotics/include/
    interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/

    interfaces/cpp-interface-json11/include/
    interfaces/cpp-interface-json11/dropbox/json11/

    interfaces/cpp-interface-coppeliasim/include/

    interfaces/cpp-interface-coppeliasim-zmq/include/
    interfaces/cpp-interface-coppeliasim-zmq/submodules/zmqRemoteApi/clients/cpp/
    interfaces/cpp-interface-coppeliasim-zmq/submodules/jsoncons/include/
)
add_subdirectory(pybind11)
pybind11_add_module(_dqrobotics

    src/dqrobotics_module.cpp

    cpp/src/DQ.cpp
    src/DQ_py.cpp

    #Internal
    cpp/src/internal/_dq_linesegment.cpp

    #Utils
    cpp/src/utils/DQ_LinearAlgebra.cpp
    src/utils/DQ_LinearAlgebra_py.cpp
    cpp/src/utils/DQ_Geometry.cpp
    src/utils/DQ_Geometry_py.cpp
    cpp/src/utils/DQ_Math.cpp
    src/utils/DQ_Math_py.cpp

    #robot_modeling
    cpp/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
    src/robot_modeling/DQ_CooperativeDualTaskSpace_py.cpp
    cpp/src/robot_modeling/DQ_Kinematics.cpp
    src/robot_modeling/DQ_Kinematics_py.cpp
    cpp/src/robot_modeling/DQ_SerialManipulator.cpp
    src/robot_modeling/DQ_SerialManipulator_py.cpp
    cpp/src/robot_modeling/DQ_SerialManipulatorDH.cpp
    src/robot_modeling/DQ_SerialManipulatorDH_py.cpp
    cpp/src/robot_modeling/DQ_SerialManipulatorMDH.cpp
    src/robot_modeling/DQ_SerialManipulatorMDH_py.cpp
    cpp/src/robot_modeling/DQ_SerialManipulatorDenso.cpp
    src/robot_modeling/DQ_SerialManipulatorDenso_py.cpp
    cpp/src/robot_modeling/DQ_MobileBase.cpp
    src/robot_modeling/DQ_MobileBase_py.cpp
    cpp/src/robot_modeling/DQ_HolonomicBase.cpp
    src/robot_modeling/DQ_HolonomicBase_py.cpp
    cpp/src/robot_modeling/DQ_DifferentialDriveRobot.cpp
    src/robot_modeling/DQ_DifferentialDriveRobot_py.cpp
    cpp/src/robot_modeling/DQ_WholeBody.cpp
    src/robot_modeling/DQ_WholeBody_py.cpp
    cpp/src/robot_modeling/DQ_SerialWholeBody.cpp
    src/robot_modeling/DQ_SerialWholeBody_py.cpp

    #robot_control
    cpp/src/robot_control/DQ_KinematicController.cpp
    src/robot_control/DQ_KinematicController_py.cpp
    cpp/src/robot_control/DQ_PseudoinverseController.cpp
    src/robot_control/DQ_PseudoinverseController_py.cpp
    cpp/src/robot_control/DQ_NumericalFilteredPseudoInverseController.cpp
    src/robot_control/DQ_NumericalFilteredPseudoInverseController_py.cpp
    cpp/src/robot_control/DQ_KinematicConstrainedController.cpp
    src/robot_control/DQ_KinematicConstrainedController_py.cpp
    cpp/src/robot_control/DQ_QuadraticProgrammingController.cpp
    src/robot_control/DQ_QuadraticProgrammingController_py.cpp
    cpp/src/robot_control/DQ_ClassicQPController.cpp
    src/robot_control/DQ_ClassicQPController_py.cpp

    #solvers
    #DQ_QuadraticProgrammingSolver.h
    src/solvers/DQ_QuadraticProgrammingSolver_py.cpp

    #robots
    cpp/src/robots/Ax18ManipulatorRobot.cpp
    cpp/src/robots/BarrettWamArmRobot.cpp
    cpp/src/robots/ComauSmartSixRobot.cpp
    cpp/src/robots/KukaLw4Robot.cpp
    cpp/src/robots/KukaYoubotRobot.cpp
    cpp/src/robots/FrankaEmikaPandaRobot.cpp

    #interfaces/vrep
    interfaces/cpp-interface-vrep/src/dqrobotics/interfaces/vrep/DQ_VrepInterface.cpp
    interfaces/cpp-interface-vrep/src/dqrobotics/interfaces/vrep/DQ_VrepInterfaceMapElement.cpp
    interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApi.c
    interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.h
    interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.c
    src/interfaces/vrep/DQ_VrepInterface_py.cpp
    interfaces/cpp-interface-vrep/src/dqrobotics/interfaces/vrep/DQ_VrepRobot.cpp
    interfaces/cpp-interface-vrep/src/dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.cpp
    interfaces/cpp-interface-vrep/src/dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.cpp
    interfaces/cpp-interface-vrep/src/dqrobotics/interfaces/vrep/robots/YouBotVrepRobot.cpp
    interfaces/cpp-interface-vrep/src/dqrobotics/interfaces/vrep/robots/FrankaEmikaPandaVrepRobot.cpp
    src/interfaces/vrep/DQ_VrepRobot_py.cpp
    src/interfaces/vrep/DQ_SerialVrepRobot_py.cpp

    #interfaces/json11
    interfaces/cpp-interface-json11/src/dqrobotics/interfaces/json11/DQ_JsonReader.cpp
    src/interfaces/json11/DQ_JsonReader_py.cpp
    interfaces/cpp-interface-json11/dropbox/json11/json11.cpp

    #interfaces/copppeliasim-zmq
    interfaces/cpp-interface-coppeliasim-zmq/submodules/zmqRemoteApi/clients/cpp/RemoteAPIClient.cpp
    interfaces/cpp-interface-coppeliasim-zmq/src/dqrobotics/interfaces/coppeliasim/internal/_zmq_wrapper.cpp
    interfaces/cpp-interface-coppeliasim-zmq/src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.cpp
    src/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ_py.cpp
)

