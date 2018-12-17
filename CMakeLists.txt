cmake_minimum_required(VERSION 2.8)
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(pybind11test)

if(UNIX AND NOT APPLE)
    include_directories(/usr/include/eigen3/ ../C++/include/)
    add_subdirectory(pybind11)
    pybind11_add_module(dqrobotics dqrobotics_module.cpp ../C++/src/DQ.cpp ../C++/src/DQ_kinematics.cpp)
endif()

if(APPLE)
    message("Building not supported on this platform yet")
    #include_directories(/usr/local/include/eigen3 ../C++/include/)
    #add_subdirectory(pybind11)
    #pybind11_add_module(dqrobotics dqrobotics_module.cpp ../C++/src/DQ.cpp ../C++/src/DQ_kinematics.cpp)
endif()

if(WIN32)
    message("Building not supported on this platform yet")
endif()
