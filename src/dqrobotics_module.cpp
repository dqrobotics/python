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

PYBIND11_MODULE(_dqrobotics, m) {

    //DQ Class
    init_DQ_py(m);

    /*****************************************************
     *  Utils
     * **************************************************/
    //dqrobotics/utils/
    py::module utils_py = m.def_submodule("_utils","A submodule of dqrobotics");

    //DQ_LinearAlgebra
    init_DQ_LinearAlgebra_py(utils_py);

    //DQ_Geometry
    init_DQ_Geometry_py(utils_py);

    //DQ_Math
    init_DQ_Math_py(utils_py);

    /*****************************************************
     *  Robots Kinematic Models
     * **************************************************/
    py::module robots_py = m.def_submodule("_robots", "A submodule of dqrobotics");

    //#include <dqrobotics/robots/Ax18ManipulatorRobot.h>
    py::class_<Ax18ManipulatorRobot> ax18manipulatorrobot_py(robots_py, "Ax18ManipulatorRobot");
    ax18manipulatorrobot_py.def_static("kinematics",&Ax18ManipulatorRobot::kinematics,"Returns the kinematics of the Ax18ManipulatorRobot");

    //#include <dqrobotics/robots/BarrettWamArmRobot.h>
    py::class_<BarrettWamArmRobot> barrettwamarmrobot_py(robots_py, "BarrettWamArmRobot");
    barrettwamarmrobot_py.def_static("kinematics",&BarrettWamArmRobot::kinematics,"Returns the kinematics of the BarrettWamArmRobot");

    //#include <dqrobotics/robots/ComauSmartSixRobot.h>
    py::class_<ComauSmartSixRobot> comausmartsixrobot_py(robots_py, "ComauSmartSixRobot");
    comausmartsixrobot_py.def_static("kinematics",&ComauSmartSixRobot::kinematics,"Returns the kinematics of the ComauSmartSixRobot");

    //#include <dqrobotics/robots/KukaLw4Robot.h>
    py::class_<KukaLw4Robot> kukalw4robot_py(robots_py, "KukaLw4Robot");
    kukalw4robot_py.def_static("kinematics",&KukaLw4Robot::kinematics,"Returns the kinematics of the KukaLw4Robot");

    //#include <dqrobotics/robots/KukaYoubotRobot.h>
    py::class_<KukaYoubotRobot> kukayoubotrobot_py(robots_py, "KukaYoubotRobot");
    kukayoubotrobot_py.def_static("kinematics",&KukaYoubotRobot::kinematics,"Returns the kinematics of the KukaYoubotRobot");

    //#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
    py::class_<FrankaEmikaPandaRobot> frankaemikapandarobot_py(robots_py, "FrankaEmikaPandaRobot");
    frankaemikapandarobot_py.def_static("kinematics",&FrankaEmikaPandaRobot::kinematics,"Returns the kinematics of the FrankaEmikaPandaRobot");

    /*****************************************************
     *  Robot Modeling <dqrobotics/robot_modeling/...>
     * **************************************************/
    py::module robot_modeling = m.def_submodule("_robot_modeling", "The robot_modeling submodule of dqrobotics");

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
     *  Solvers <dqrobotics/solvers/...>
     * **************************************************/
    py::module solvers = m.def_submodule("_solvers", "The solvers submodule of dqrobotics");

    //DQ_QuadraticProgrammingSolver
    init_DQ_QuadraticProgrammingSolver_py(solvers);

    /*****************************************************
     *  Robot Control <dqrobotics/robot_control/...>
     * **************************************************/
    py::module robot_control = m.def_submodule("_robot_control", "The robot_control submodule of dqrobotics");

    py::enum_<ControlObjective>(robot_control, "ControlObjective")
            .value("Line",           ControlObjective::Line)
            .value("None",           ControlObjective::None)
            .value("Pose",           ControlObjective::Pose)
            .value("Plane",          ControlObjective::Plane)
            .value("Distance",       ControlObjective::Distance)
            .value("DistanceToPlane",ControlObjective::DistanceToPlane)
            .value("Rotation",       ControlObjective::Rotation)
            .value("Translation",    ControlObjective::Translation)
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
    py::module interfaces_py = m.def_submodule("_interfaces", "A submodule of dqrobotics");

    /*****************************************************
     *  Json11 submodule
     * **************************************************/
    py::module json11_py = interfaces_py.def_submodule("_json11", "A submodule of dqrobotics");

    //DQ_JsonReader
    init_DQ_JsonReader_py(json11_py);

    /*****************************************************
     *  CoppeliaSim Submodule
     * **************************************************/
    py::module coppeliasim_py = interfaces_py.def_submodule("_coppeliasim", "The CoppeliaSim submodule of DQ Robotics.");

    //DQ_CoppeliaSimInterface
    init_DQ_CoppeliaSimInterface_py(coppeliasim_py);

    //DQ_CoppeliaSimInterfaceZMQ
    init_DQ_CoppeliaSimInterfaceZMQ_py(coppeliasim_py);
}

