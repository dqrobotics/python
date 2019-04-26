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
        dqrobotics_module.cpp

        cpp/src/DQ.cpp

        cpp/src/utils/DQ_LinearAlgebra.cpp
        cpp/src/utils/DQ_Geometry.cpp

        cpp/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
        cpp/src/robot_modeling/DQ_Kinematics.cpp
        cpp/src/robot_modeling/DQ_SerialManipulator.cpp
        cpp/src/robot_modeling/DQ_MobileBase.cpp
        cpp/src/robot_modeling/DQ_HolonomicBase.cpp
        cpp/src/robot_modeling/DQ_DifferentialDriveRobot.cpp
        cpp/src/robot_modeling/DQ_WholeBody.cpp

        cpp/src/robots/Ax18ManipulatorRobot.cpp
        cpp/src/robots/BarrettWamArmRobot.cpp
        cpp/src/robots/ComauSmartSixRobot.cpp
        cpp/src/robots/KukaLw4Robot.cpp

        #VREP INTERFACE
        interfaces/cpp-interface-vrep/src/vrep_interface.cpp
        interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApi.c
        interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.h
        interfaces/cpp-interface-vrep/coppeliarobotics/remoteApi/extApiPlatform.c
        )
endif()

if(APPLE)
    include_directories(/usr/local/include/eigen3 cpp/include/)
    add_subdirectory(pybind11)
    pybind11_add_module(dqrobotics
        dqrobotics_module.cpp

        cpp/src/DQ.cpp

        cpp/src/utils/DQ_LinearAlgebra.cpp
        cpp/src/utils/DQ_Geometry.cpp

        cpp/src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
        cpp/src/robot_modeling/DQ_Kinematics.cpp
        cpp/src/robot_modeling/DQ_SerialManipulator.cpp
        cpp/src/robot_modeling/DQ_MobileBase.cpp
        cpp/src/robot_modeling/DQ_HolonomicBase.cpp
        cpp/src/robot_modeling/DQ_DifferentialDriveRobot.cpp
        cpp/src/robot_modeling/DQ_WholeBody.cpp

        cpp/src/robots/Ax18ManipulatorRobot.cpp
        cpp/src/robots/BarrettWamArmRobot.cpp
        cpp/src/robots/ComauSmartSixRobot.cpp
        cpp/src/robots/KukaLw4Robot.cpp
        )
endif()

if(WIN32)
    message("Building not supported on this platform yet")
endif()
