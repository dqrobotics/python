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
 * @brief Binds `DQ_SerialManipulator`, the abstract serial-manipulator base
 * class that extends `DQ_Kinematics` with common fixed-base and mobile-base
 * serial-chain operations, to the Python module @p m.
 */
void init_DQ_SerialManipulator_py(py::module& m)
{
    py::class_<
            DQ_SerialManipulator,
            std::shared_ptr<DQ_SerialManipulator>,
            DQ_Kinematics
            > dqserialmanipulator_py(
                m,
                "DQ_SerialManipulator",
                "Abstract class that defines serial manipulators. It extends DQ_Kinematics with the common operations of fixed-base and mobile-base serial chains while subclasses implement the raw forward kinematics and raw Jacobians for a specific parameterization.");

    dqserialmanipulator_py.def(
        "get_effector",
        &DQ_SerialManipulator::get_effector,
        "Returns the current end-effector rigid transformation appended to the last link.");
    dqserialmanipulator_py.def(
        "set_effector",
        &DQ_SerialManipulator::set_effector,
        py::arg("new_effector"),
        "Sets the current end-effector rigid transformation from the last link to the tool frame.");
    dqserialmanipulator_py.def(
        "get_lower_q_limit",
        &DQ_SerialManipulator::get_lower_q_limit,
        "Returns the lower joint-position limits.");
    dqserialmanipulator_py.def(
        "set_lower_q_limit",
        &DQ_SerialManipulator::set_lower_q_limit,
        py::arg("lower_q_limit"),
        "Sets the lower joint-position limits.");
    dqserialmanipulator_py.def(
        "get_lower_q_dot_limit",
        &DQ_SerialManipulator::get_lower_q_dot_limit,
        "Returns the lower joint-velocity limits.");
    dqserialmanipulator_py.def(
        "set_lower_q_dot_limit",
        &DQ_SerialManipulator::set_lower_q_dot_limit,
        py::arg("lower_q_dot_limit"),
        "Sets the lower joint-velocity limits.");
    dqserialmanipulator_py.def(
        "get_upper_q_limit",
        &DQ_SerialManipulator::get_upper_q_limit,
        "Returns the upper joint-position limits.");
    dqserialmanipulator_py.def(
        "set_upper_q_limit",
        &DQ_SerialManipulator::set_upper_q_limit,
        py::arg("upper_q_limit"),
        "Sets the upper joint-position limits.");
    dqserialmanipulator_py.def(
        "get_upper_q_dot_limit",
        &DQ_SerialManipulator::get_upper_q_dot_limit,
        "Returns the upper joint-velocity limits.");
    dqserialmanipulator_py.def(
        "set_upper_q_dot_limit",
        &DQ_SerialManipulator::set_upper_q_dot_limit,
        py::arg("upper_q_dot_limit"),
        "Sets the upper joint-velocity limits.");

    dqserialmanipulator_py.def(
        "raw_fkm",
        (DQ (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::raw_fkm,
        py::arg("q_vec"),
        "Computes the raw forward kinematics up to the last link and returns the pose before applying the reference frame and the end effector.");
    dqserialmanipulator_py.def(
        "raw_fkm",
        (DQ (DQ_SerialManipulator::*)(const VectorXd&, const int&) const)&DQ_SerialManipulator::raw_fkm,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the raw forward kinematics up to the requested link and returns the pose before applying the reference frame and the end effector.");
    dqserialmanipulator_py.def(
        "raw_pose_jacobian",
        (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::raw_pose_jacobian,
        py::arg("q_vec"),
        "Computes the raw pose Jacobian up to the last link, without reference-frame or end-effector transformations.");
    dqserialmanipulator_py.def(
        "raw_pose_jacobian",
        (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const int&) const)&DQ_SerialManipulator::raw_pose_jacobian,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the raw pose Jacobian up to the requested link, without reference-frame or end-effector transformations.");
    dqserialmanipulator_py.def(
        "raw_pose_jacobian_derivative",
        (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const VectorXd&) const)&DQ_SerialManipulator::raw_pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        "Computes the time derivative of the raw pose Jacobian up to the last link.");
    dqserialmanipulator_py.def(
        "raw_pose_jacobian_derivative",
        (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_SerialManipulator::raw_pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_ith_link"),
        "Computes the time derivative of the raw pose Jacobian up to the requested link.");

    dqserialmanipulator_py.def(
        "fkm",
        (DQ (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::fkm,
        py::arg("q_vec"),
        "Computes the forward kinematics of the end effector, including the reference frame and the stored end-effector rigid transformation.");
    dqserialmanipulator_py.def(
        "fkm",
        (DQ (DQ_SerialManipulator::*)(const VectorXd&,const int&) const)&DQ_SerialManipulator::fkm,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the forward kinematics up to a given link, including the reference frame and applying the stored end-effector transformation only when the requested link is the last one.");

    dqserialmanipulator_py.def(
        "get_dim_configuration_space",
        &DQ_SerialManipulator::get_dim_configuration_space,
        "Returns the dimension of the configuration space as the number of generalized coordinates of the serial manipulator.");

    dqserialmanipulator_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const int&) const)&DQ_SerialManipulator::pose_jacobian,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the pose Jacobian up to a given link. The returned Jacobian includes the reference frame and applies the stored end-effector transformation only when the requested link is the last one.");
    dqserialmanipulator_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::pose_jacobian,
        py::arg("q_vec"),
        "Computes the pose Jacobian of the end effector so that vec8(pose_dot) = J * q_dot.");
    dqserialmanipulator_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_SerialManipulator::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_ith_link"),
        "Computes the time derivative of the pose Jacobian up to a given link, including the reference frame and applying the stored end-effector transformation only when the requested link is the last one.");
    dqserialmanipulator_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const VectorXd&) const)&DQ_SerialManipulator::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        "Computes the time derivative of the pose Jacobian of the end effector.");
}
