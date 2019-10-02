cmake_minimum_required(VERSION 2.8)
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(dqroboticspython)

#VREP INTERFACE
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)

if(UNIX AND NOT APPLE)
    include_directories(
        /usr/include/eigen3/
        cpp/include/
        interfaces/cpp-interface-vrep/include/
        interfaces/cpp-interface-vrep/coppeliarobotics/include
        interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi
        )
    add_subdirectory(pybind11)
    pybind11_add_module(dqrobotics
        src/dqrobotics_module.cpp

        cpp/src/DQ.cpp
        src/DQ_py.cpp

        #Utils
        cpp/src/utils/DQ_LinearAlgebra.cpp
        src/utils/DQ_LinearAlgebra_py.cpp
        cpp/src/utils/DQ_Geometry.cpp
        src/utils/DQ_Geometry_py.cpp

        #robot_modeling
        cpp/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
        src/robot_modeling/DQ_CooperativeDualTaskSpace_py.cpp
        cpp/src/robot_modeling/DQ_Kinematics.cpp
        src/robot_modeling/DQ_Kinematics_py.cpp
        cpp/src/robot_modeling/DQ_SerialManipulator.cpp
        src/robot_modeling/DQ_SerialManipulator_py.cpp
        cpp/src/robot_modeling/DQ_MobileBase.cpp
        src/robot_modeling/DQ_MobileBase_py.cpp
        cpp/src/robot_modeling/DQ_HolonomicBase.cpp
        src/robot_modeling/DQ_HolonomicBase_py.cpp
        cpp/src/robot_modeling/DQ_DifferentialDriveRobot.cpp
        src/robot_modeling/DQ_DifferentialDriveRobot_py.cpp
        cpp/src/robot_modeling/DQ_WholeBody.cpp
        src/robot_modeling/DQ_WholeBody_py.cpp

        #robot_control
        cpp/src/robot_control/DQ_KinematicController.cpp
        src/robot_control/DQ_KinematicController_py.cpp
        cpp/src/robot_control/DQ_TaskSpacePseudoInverseController.cpp
        src/robot_control/DQ_TaskSpacePseudoInverseController_py.cpp
        cpp/src/robot_control/DQ_KinematicConstrainedController.cpp
        src/robot_control/DQ_KinematicConstrainedController_py.cpp
        cpp/src/robot_control/DQ_TaskspaceQuadraticProgrammingController.cpp
        src/robot_control/DQ_TaskspaceQuadraticProgrammingController_py.cpp
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

        #interfaces/vrep
        interfaces/cpp-interface-vrep/src/DQ_VrepInterface.cpp
        interfaces/cpp-interface-vrep/src/DQ_VrepInterfaceMapElement.cpp
        interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApi.c
        interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.h
        interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.c
        src/interfaces/vrep/DQ_VrepInterface_py.cpp
        interfaces/cpp-interface-vrep/src/DQ_VrepRobot.cpp
        interfaces/cpp-interface-vrep/src/LBR4pVrepRobot.cpp
        interfaces/cpp-interface-vrep/src/YouBotVrepRobot.cpp
        src/interfaces/vrep/DQ_VrepRobot_py.cpp
        )
endif()

if(APPLE)
    message("Building not supported on this platform.")
endif()

if(WIN32)
    message("Building not supported on this platform.")
endif()
