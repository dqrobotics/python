/**
(C) Copyright 2019 DQ Robotics Developers

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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#ifndef DQ_ROBOTICS_PYTHON_HEADER_GUARD
#define DQ_ROBOTICS_PYTHON_HEADER_GUARD

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

#include <dqrobotics/DQ.h>

#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <dqrobotics/utils/DQ_Math.h>

#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_MobileBase.h>
#include <dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include <dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h>
#include <dqrobotics/robot_modeling/DQ_WholeBody.h>
#include <dqrobotics/robot_modeling/DQ_SerialWholeBody.h>

#include <dqrobotics/robot_control/DQ_KinematicController.h>
#include <dqrobotics/robot_control/DQ_KinematicConstrainedController.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <dqrobotics/robot_control/DQ_QuadraticProgrammingController.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

#include <dqrobotics/solvers/DQ_QuadraticProgrammingSolver.h>

#include <dqrobotics/robots/Ax18ManipulatorRobot.h>
#include <dqrobotics/robots/BarrettWamArmRobot.h>
#include <dqrobotics/robots/ComauSmartSixRobot.h>
#include <dqrobotics/robots/KukaLw4Robot.h>
#include <dqrobotics/robots/KukaYoubotRobot.h>

#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepRobot.h>
#include <dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.h>
#include <dqrobotics/interfaces/vrep/robots/YouBotVrepRobot.h>

#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

using namespace DQ_robotics;
using namespace Eigen;

//DQ
void init_DQ_py(py::module& m);

//dqrobotics/utils
void init_DQ_LinearAlgebra_py(py::module& m);
void init_DQ_Geometry_py(py::module& m);

//dqrobotics/robot_modeling
void init_DQ_Kinematics_py(py::module& m);
void init_DQ_SerialManipulator_py(py::module& m);
void init_DQ_SerialManipulatorDH_py(py::module& m);
void init_DQ_MobileBase_py(py::module& m);
void init_DQ_HolonomicBase_py(py::module& m);
void init_DQ_DifferentialDriveRobot_py(py::module& m);
void init_DQ_CooperativeDualTaskSpace_py(py::module& m);
void init_DQ_WholeBody_py(py::module& m);
void init_DQ_SerialWholeBody_py(py::module& m);

//dqrobotics/robot_control
void init_DQ_ClassicQPController_py(py::module& m);
void init_DQ_KinematicConstrainedController_py(py::module& m);
void init_DQ_KinematicController_py(py::module& m);
void init_DQ_PseudoinverseController_py(py::module& m);
void init_DQ_QuadraticProgrammingController_py(py::module& m);

//dqrobotics/solvers
void init_DQ_QuadraticProgrammingSolver_py(py::module& m);

//dqrobotics/interfaces/vrep
void init_DQ_VrepInterface_py(py::module& m);
void init_DQ_VrepRobot_py(py::module& m);

//dqrobotics/interfaces/json11
void init_DQ_JsonReader_py(py::module& m);

#endif
