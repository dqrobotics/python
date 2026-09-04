/**
(C) Copyright 2020-2023 DQ Robotics Developers

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
1. Murilo M. Marinho (murilomarinho@ieee.org)
   - Responsible for the original implementation.

2. Juan Jose Quiroz Omana (juanjqogm@gmail.com)
   - Added the two raw_fkm methods to fix issue 54 (https://github.com/dqrobotics/python/issues/54)
*/

#include "../dqrobotics_module.h"

/**
 * @brief Binds `DQ_SerialWholeBody`, the robot model composed of multiple
 * serially coupled kinematic chains with a single combined link index, to the
 * Python module @p m.
 */
void init_DQ_SerialWholeBody_py(py::module& m)
{
    py::class_<
            DQ_SerialWholeBody,
            std::shared_ptr<DQ_SerialWholeBody>,
            DQ_Kinematics
            > dqserialwholebody_py(
                m,
                "DQ_SerialWholeBody",
                "Robot model composed of multiple serially coupled kinematic chains. DQ_SerialWholeBody concatenates several DQ_Kinematics objects and exposes a single combined link index across the whole serial composition.");
    dqserialwholebody_py.def(
        py::init<std::shared_ptr<DQ_Kinematics>>(),
        py::arg("robot"),
        "Constructs a serial whole-body model from its first chain.");
    dqserialwholebody_py.def(
        "add",
        &DQ_SerialWholeBody::add,
        py::arg("robot"),
        "Appends a new chain to the end of the serial whole-body model.");
    dqserialwholebody_py.def(
        "fkm",
        (DQ (DQ_SerialWholeBody::*)(const VectorXd&) const)&DQ_SerialWholeBody::fkm,
        py::arg("q"),
        "Computes the forward kinematics of the complete serial whole-body model, including the reference frame.");
    dqserialwholebody_py.def(
        "fkm",
        (DQ (DQ_SerialWholeBody::*)(const VectorXd&,const int&) const)&DQ_SerialWholeBody::fkm,
        py::arg("q"),
        py::arg("to_ith_link"),
        "Computes the forward kinematics up to a combined link index, including the reference frame.");
    dqserialwholebody_py.def(
        "raw_fkm",
        (DQ (DQ_SerialWholeBody::*)(const VectorXd&) const)&DQ_SerialWholeBody::raw_fkm,
        py::arg("q"),
        "Computes the raw forward kinematics of the complete serial whole-body model without the reference frame.");
    dqserialwholebody_py.def(
        "raw_fkm",
        (DQ (DQ_SerialWholeBody::*)(const VectorXd&, const int&) const)&DQ_SerialWholeBody::raw_fkm,
        py::arg("q"),
        py::arg("to_ith_link"),
        "Computes the raw forward kinematics up to a combined link index without the reference frame.");
    dqserialwholebody_py.def(
        "get_dim_configuration_space",
        &DQ_SerialWholeBody::get_dim_configuration_space,
        "Returns the dimension of the configuration space of the serial whole-body model.");
    dqserialwholebody_py.def(
        "get_chain",
        &DQ_SerialWholeBody::get_chain,
        py::arg("to_ith_chain"),
        "Returns a raw pointer to one of the stored chains.");
    dqserialwholebody_py.def(
        "get_chain_as_serial_manipulator_dh",
        &DQ_SerialWholeBody::get_chain_as_serial_manipulator_dh,
        py::arg("to_ith_chain"),
        "Returns a copy of the selected chain as a DQ_SerialManipulatorDH.");
    dqserialwholebody_py.def(
        "get_chain_as_holonomic_base",
        &DQ_SerialWholeBody::get_chain_as_holonomic_base,
        py::arg("to_ith_chain"),
        "Returns a copy of the selected chain as a DQ_HolonomicBase.");
    dqserialwholebody_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_SerialWholeBody::*)(const VectorXd&, const int&) const)&DQ_SerialWholeBody::pose_jacobian,
        py::arg("q"),
        py::arg("to_ith_link"),
        "Computes the pose Jacobian up to a combined link index.");
    dqserialwholebody_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_SerialWholeBody::*)(const VectorXd&) const)&DQ_SerialWholeBody::pose_jacobian,
        py::arg("q"),
        "Computes the pose Jacobian of the complete serial whole-body model.");
    dqserialwholebody_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_SerialWholeBody::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_SerialWholeBody::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_ith_link"),
        "Computes the time derivative of the pose Jacobian up to a combined link index. This method is currently not implemented and always throws.");
    dqserialwholebody_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_SerialWholeBody::*)(const VectorXd&, const VectorXd&) const)&DQ_SerialWholeBody::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        "Computes the time derivative of the pose Jacobian of the complete serial whole-body model. This method is currently not implemented and always throws.");
}
