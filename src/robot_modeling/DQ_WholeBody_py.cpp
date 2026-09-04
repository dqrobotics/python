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
 * @brief Binds `DQ_WholeBody`, the robot model composed of multiple kinematic
 * chains connected in series and treated as subchains, to the Python module
 * @p m.
 */
void init_DQ_WholeBody_py(py::module& m)
{
    py::class_<
            DQ_WholeBody,
            std::shared_ptr<DQ_WholeBody>,
            DQ_Kinematics
            > dqwholebody_py(
                m,
                "DQ_WholeBody",
                "Robot model composed of multiple kinematic chains connected in series. DQ_WholeBody concatenates several DQ_Kinematics objects and treats each one as a whole subchain.");
    dqwholebody_py.def(
        py::init<std::shared_ptr<DQ_Kinematics>>(),
        py::arg("robot"),
        "Constructs a whole-body model from its first chain.");
    dqwholebody_py.def(
        "add",
        &DQ_WholeBody::add,
        py::arg("robot"),
        "Appends a new chain to the end of the whole-body model.");
    dqwholebody_py.def(
        "fkm",
        (DQ (DQ_WholeBody::*)(const VectorXd&) const)&DQ_WholeBody::fkm,
        py::arg("q"),
        "Computes the forward kinematics of the complete whole-body model, including the reference frame.");
    dqwholebody_py.def(
        "fkm",
        (DQ (DQ_WholeBody::*)(const VectorXd&,const int&) const)&DQ_WholeBody::fkm,
        py::arg("q"),
        py::arg("to_chain"),
        "Computes the forward kinematics up to a given chain, stopping the computation at the requested subchain and including the reference frame.");
    dqwholebody_py.def(
        "get_dim_configuration_space",
        &DQ_WholeBody::get_dim_configuration_space,
        "Returns the dimension of the configuration space of the whole-body model.");
    dqwholebody_py.def(
        "get_chain",
        &DQ_WholeBody::get_chain,
        py::arg("to_ith_chain"),
        "Returns a raw pointer to one of the stored chains.");
    dqwholebody_py.def(
        "get_chain_as_serial_manipulator_dh",
        &DQ_WholeBody::get_chain_as_serial_manipulator_dh,
        py::arg("to_ith_chain"),
        "Returns a copy of the selected chain as a DQ_SerialManipulatorDH.");
    dqwholebody_py.def(
        "get_chain_as_holonomic_base",
        &DQ_WholeBody::get_chain_as_holonomic_base,
        py::arg("to_ith_chain"),
        "Returns a copy of the selected chain as a DQ_HolonomicBase.");
    dqwholebody_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_WholeBody::*)(const VectorXd&, const int&) const)&DQ_WholeBody::pose_jacobian,
        py::arg("q"),
        py::arg("to_ith_chain"),
        "Computes the pose Jacobian up to a given chain.");
    dqwholebody_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_WholeBody::*)(const VectorXd&) const)&DQ_WholeBody::pose_jacobian,
        py::arg("q"),
        "Computes the pose Jacobian of the complete whole-body model.");
    dqwholebody_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_WholeBody::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_WholeBody::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_ith_link"),
        "Computes the time derivative of the pose Jacobian. This method is currently not implemented and always throws.");
    dqwholebody_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_WholeBody::*)(const VectorXd&, const VectorXd&) const)&DQ_WholeBody::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        "Computes the time derivative of the pose Jacobian of the complete whole-body model. This method is currently not implemented and always throws.");
}
