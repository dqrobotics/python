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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#include "../dqrobotics_module.h"

/**
 * @brief Binds `DQ_DifferentialDriveRobot`, the basic differential-drive
 * mobile-robot implementation whose pose Jacobians account for the
 * nonholonomic rolling constraint, to the Python module @p m.
 */
void init_DQ_DifferentialDriveRobot_py(py::module& m)
{
    py::class_<
            DQ_DifferentialDriveRobot,
            std::shared_ptr<DQ_DifferentialDriveRobot>,
            DQ_HolonomicBase
            > dqdifferentialdriverobot_py(
                m,
                "DQ_DifferentialDriveRobot",
                "Basic implementation of a differential-drive mobile robot. The robot pose is modeled as a holonomic base with configuration q = [x, y, phi]^T, while the actuation is described by the angular velocities of the right and left wheels.");
    dqdifferentialdriverobot_py.def(
        py::init<const double&, const double&>(),
        py::arg("wheel_radius"),
        py::arg("distance_between_wheels"),
        "Constructs a differential-drive robot from the wheel radius and the distance between the wheels.");
    dqdifferentialdriverobot_py.def(
        "constraint_jacobian",
        &DQ_DifferentialDriveRobot::constraint_jacobian,
        py::arg("phi"),
        "Computes the constraint Jacobian relating the right- and left-wheel angular velocities to [x_dot, y_dot, phi_dot]^T.");
    dqdifferentialdriverobot_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_DifferentialDriveRobot::*)(const VectorXd&, const int&) const)&DQ_DifferentialDriveRobot::pose_jacobian,
        py::arg("q"),
        py::arg("to_link"),
        "Computes the constrained pose Jacobian up to the requested column.");
    dqdifferentialdriverobot_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_DifferentialDriveRobot::*)(const VectorXd&) const)&DQ_DifferentialDriveRobot::pose_jacobian,
        py::arg("q"),
        "Computes the full constrained pose Jacobian.");
    dqdifferentialdriverobot_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_DifferentialDriveRobot::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_DifferentialDriveRobot::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_link"),
        "Computes the time derivative of the constrained pose Jacobian up to the requested column.");
    dqdifferentialdriverobot_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_DifferentialDriveRobot::*)(const VectorXd&, const VectorXd&) const)&DQ_DifferentialDriveRobot::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        "Computes the full time derivative of the constrained pose Jacobian.");
}
