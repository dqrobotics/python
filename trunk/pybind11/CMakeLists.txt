cmake_minimum_required(VERSION 2.8)
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(pybind11test)
include_directories(
     /usr/include/eigen3/)

#add_executable(${PROJECT_NAME} "main.cpp")
#target_link_libraries(${PROJECT_NAME} dqrobotics)

add_subdirectory(pybind11)
pybind11_add_module(dqrobotics dqrobotics_module.cpp /usr/src/dqrobotics/DQ.cpp /usr/src/dqrobotics/DQ_kinematics.cpp)
