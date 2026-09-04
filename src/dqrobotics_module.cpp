/**
(C) Copyright 2019-2023 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#include "dqrobotics_module.h"

/**
 * @brief Defines the `_dqrobotics` Python extension module and registers all
 * of its classes, functions, enumerations, and submodules.
 */
PYBIND11_MODULE(_dqrobotics, m) {

    //DQ Class
    init_DQ_py(m);

    /*****************************************************
     *  Utils
     * **************************************************/
    //dqrobotics/utils/
    py::module utils_py = m.def_submodule("_utils","Linear-algebra, geometric, and mathematical utilities used throughout dqrobotics.");

    //DQ_LinearAlgebra
    init_DQ_LinearAlgebra_py(utils_py);

    //DQ_Geometry
    init_DQ_Geometry_py(utils_py);

    //DQ_Math
    init_DQ_Math_py(utils_py);

    /*****************************************************
     *  Robot Modeling <dqrobotics/robot_modeling/...>
     * **************************************************/
    py::module robot_modeling = m.def_submodule("_robot_modeling", "Kinematic models of serial, mobile, cooperative dual-arm, and whole-body robots.");

    //DQ_Kinematics
    init_DQ_Kinematics_py(robot_modeling);

    //DQ_SerialManipulator
    init_DQ_SerialManipulator_py(robot_modeling);

    //DQ_SerialManipulatorDH
    init_DQ_SerialManipulatorDH_py(robot_modeling);
    
    //DQ_SerialManipulatorMDH
    init_DQ_SerialManipulatorMDH_py(robot_modeling);

    //DQ_SerialManipulatorDenso
    init_DQ_SerialManipulatorDenso_py(robot_modeling);

    //DQ_CooperativeDualTaskSpace
    init_DQ_CooperativeDualTaskSpace_py(robot_modeling);

    //DQ_MobileBase
    init_DQ_MobileBase_py(robot_modeling);

    //DQ_HolonomicBase
    init_DQ_HolonomicBase_py(robot_modeling);

    //DQ_DifferentialDriveRobot
    init_DQ_DifferentialDriveRobot_py(robot_modeling);

    //DQ_WholeBody
    init_DQ_WholeBody_py(robot_modeling);

    //DQ_SerialWholeBody
    init_DQ_SerialWholeBody_py(robot_modeling);

/*****************************************************
     *  Robots Kinematic Models
     * **************************************************/
    py::module robots_py = m.def_submodule("_robots", "Ready-to-use kinematic models of well-known commercial robot manipulators.");

    //#include <dqrobotics/robots/Ax18ManipulatorRobot.h>
    py::class_<Ax18ManipulatorRobot> ax18manipulatorrobot_py(robots_py, "Ax18ManipulatorRobot",
                                                              "Provides the kinematic model of the AX-18 manipulator arm.");
    ax18manipulatorrobot_py.def_static("kinematics",&Ax18ManipulatorRobot::kinematics,"Returns the kinematic model of the AX-18 manipulator arm.");

    //#include <dqrobotics/robots/BarrettWamArmRobot.h>
    py::class_<BarrettWamArmRobot> barrettwamarmrobot_py(robots_py, "BarrettWamArmRobot",
                                                          "Provides the kinematic model of the Barrett WAM arm robot manipulator.");
    barrettwamarmrobot_py.def_static("kinematics",&BarrettWamArmRobot::kinematics,"Returns the kinematic model of the Barrett WAM arm robot manipulator.");

    //#include <dqrobotics/robots/ComauSmartSixRobot.h>
    py::class_<ComauSmartSixRobot> comausmartsixrobot_py(robots_py, "ComauSmartSixRobot",
                                                          "Provides the kinematic model of the COMAU SmartSiX robot manipulator.");
    comausmartsixrobot_py.def_static("kinematics",&ComauSmartSixRobot::kinematics,"Returns the kinematic model of the COMAU SmartSiX robot manipulator.");

    //#include <dqrobotics/robots/KukaLw4Robot.h>
    py::class_<KukaLw4Robot> kukalw4robot_py(robots_py, "KukaLw4Robot",
                                              "Provides the kinematic model of the KUKA LWR4 robot manipulator.");
    kukalw4robot_py.def_static("kinematics",&KukaLw4Robot::kinematics,"Returns the kinematic model of the KUKA LWR4 robot manipulator.");

    //#include <dqrobotics/robots/KukaYoubotRobot.h>
    py::class_<KukaYoubotRobot> kukayoubotrobot_py(robots_py, "KukaYoubotRobot",
                                                    "Provides the whole-body kinematic model of the KUKA youBot mobile manipulator.");
    kukayoubotrobot_py.def_static("kinematics",&KukaYoubotRobot::kinematics,"Returns the whole-body kinematic model of the KUKA youBot mobile manipulator.");

    //#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
    py::class_<FrankaEmikaPandaRobot> frankaemikapandarobot_py(robots_py, "FrankaEmikaPandaRobot",
                                                                "Provides the kinematic model of the Franka Emika Panda robot manipulator.");
    frankaemikapandarobot_py.def_static("kinematics",&FrankaEmikaPandaRobot::kinematics,"Returns the kinematic model of the Franka Emika Panda robot, as calibrated by the manufacturer.");

    /*****************************************************
     *  Solvers <dqrobotics/solvers/...>
     * **************************************************/
    py::module solvers = m.def_submodule("_solvers", "Quadratic-programming solver interfaces used by the QP-based kinematic controllers.");

    //DQ_QuadraticProgrammingSolver
    init_DQ_QuadraticProgrammingSolver_py(solvers);

    /*****************************************************
     *  Robot Control <dqrobotics/robot_control/...>
     * **************************************************/
    py::module robot_control = m.def_submodule("_robot_control", "Kinematic controllers that drive a robot's task-space error to zero.");

    py::enum_<ControlObjective>(robot_control, "ControlObjective",
                                 "Enumerates the task-space objectives supported by DQ_KinematicController.")
            .value("Line",           ControlObjective::Line,           "Control a line primitive attached to the end-effector.")
            .value("None",           ControlObjective::None,           "No control objective has been selected yet.")
            .value("Pose",           ControlObjective::Pose,           "Control the full end-effector pose.")
            .value("Plane",          ControlObjective::Plane,          "Control a plane primitive attached to the end-effector.")
            .value("Distance",       ControlObjective::Distance,       "Control the squared distance between the end-effector translation and the origin.")
            .value("DistanceToPlane",ControlObjective::DistanceToPlane,"Control the signed distance from the end-effector point to a target plane.")
            .value("Rotation",       ControlObjective::Rotation,       "Control only the end-effector orientation.")
            .value("Translation",    ControlObjective::Translation,    "Control only the end-effector translation.")
            .export_values();

    //DQ_KinematicController
    init_DQ_KinematicController_py(robot_control);

    //DQ_TaskSpacePseudoInverseController
    init_DQ_PseudoinverseController_py(robot_control);

    //DQ_NumericalFilteredPseudoinverseController
    init_DQ_NumericalFilteredPseudoInverseController_py(robot_control);

    //DQ_KinematicConstrainedController
    init_DQ_KinematicConstrainedController_py(robot_control);

    //DQ_TaskspaceQuadraticProgrammingController
    init_DQ_QuadraticProgrammingController_py(robot_control);

    //DQ_ClassicQPController
    init_DQ_ClassicQPController_py(robot_control);

    /*****************************************************
     *  Interfaces Submodule
     * **************************************************/
    py::module interfaces_py = m.def_submodule("_interfaces", "Interfaces to third-party simulators and data formats.");

    /*****************************************************
     *  Json11 submodule
     * **************************************************/
    py::module json11_py = interfaces_py.def_submodule("_json11", "Reads dqrobotics objects (DQ, robot models) serialized as JSON using the json11 library.");

    //DQ_JsonReader
    init_DQ_JsonReader_py(json11_py);

}

