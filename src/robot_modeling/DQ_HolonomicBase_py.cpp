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
 * @brief Binds `DQ_HolonomicBase`, the basic holonomic mobile-base
 * implementation with planar configuration q = [x, y, phi]^T, to the Python
 * module @p m.
 */
void init_DQ_HolonomicBase_py(py::module& m)
{
    py::class_<
            DQ_HolonomicBase,
            std::shared_ptr<DQ_HolonomicBase>,
            DQ_MobileBase
            > dqholonomicbase_py(
                m,
                "DQ_HolonomicBase",
                "Basic implementation of a holonomic mobile base. The configuration vector is q = [x, y, phi]^T, where x and y describe the planar position and phi is the planar orientation.");
    dqholonomicbase_py.def(
        py::init(),
        "Constructs a holonomic base with configuration-space dimension three.");
    dqholonomicbase_py.def(
        "fkm",
        (DQ (DQ_HolonomicBase::*)(const VectorXd&) const)&DQ_HolonomicBase::fkm,
        py::arg("q"),
        "Computes the mobile-base pose while considering the frame displacement.");
    dqholonomicbase_py.def(
        "fkm",
        (DQ (DQ_HolonomicBase::*)(const VectorXd&,const int&) const)&DQ_HolonomicBase::fkm,
        py::arg("q"),
        py::arg("to_ith_link"),
        "Computes the mobile-base pose while considering the frame displacement. This compatibility overload accepts only to_ith_link = 2.");
    dqholonomicbase_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_HolonomicBase::*)(const VectorXd&, const int&) const)&DQ_HolonomicBase::pose_jacobian,
        py::arg("q"),
        py::arg("to_link"),
        "Computes the pose Jacobian while considering the frame displacement up to the requested column.");
    dqholonomicbase_py.def(
        "pose_jacobian",
        (MatrixXd (DQ_HolonomicBase::*)(const VectorXd&) const)&DQ_HolonomicBase::pose_jacobian,
        py::arg("q"),
        "Computes the full pose Jacobian while considering the frame displacement.");
    dqholonomicbase_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_HolonomicBase::*)(const VectorXd&, const VectorXd&) const)&DQ_HolonomicBase::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        "Computes the full time derivative of the pose Jacobian.");
    dqholonomicbase_py.def(
        "pose_jacobian_derivative",
        (MatrixXd (DQ_HolonomicBase::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_HolonomicBase::pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_link"),
        "Computes the time derivative of the pose Jacobian while considering the frame displacement up to the requested column.");
    dqholonomicbase_py.def(
        "get_dim_configuration_space",
        &DQ_HolonomicBase::get_dim_configuration_space,
        "Returns the dimension of the configuration space. For a holonomic base, q = [x, y, phi]^T.");
    dqholonomicbase_py.def(
        "raw_fkm",
        &DQ_HolonomicBase::raw_fkm,
        py::arg("q"),
        "Computes the planar mobile-base pose without considering the frame displacement.");
    dqholonomicbase_py.def(
        "raw_pose_jacobian",
        &DQ_HolonomicBase::raw_pose_jacobian,
        py::arg("q"),
        py::arg("to_link") = 2,
        "Computes the raw pose Jacobian of the planar mobile base up to the requested column.");
    dqholonomicbase_py.def(
        "raw_pose_jacobian_derivative",
        &DQ_HolonomicBase::raw_pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_link") = 2,
        "Computes the raw time derivative of the pose Jacobian of the planar mobile base up to the requested column.");
}
