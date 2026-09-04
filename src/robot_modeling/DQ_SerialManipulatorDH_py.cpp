/**
(C) Copyright 2020 DQ Robotics Developers

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
 * @brief Binds `DQ_SerialManipulatorDH`, the concrete serial manipulator based
 * on the standard Denavit-Hartenberg convention, to the Python module @p m.
 */
void init_DQ_SerialManipulatorDH_py(py::module& m)
{
    py::class_<
            DQ_SerialManipulatorDH,
            std::shared_ptr<DQ_SerialManipulatorDH>,
            DQ_SerialManipulator
            > dqserialmanipulatordh_py(
                m,
                "DQ_SerialManipulatorDH",
                "Concrete serial manipulator based on the standard Denavit-Hartenberg convention. The constructor expects a 5 x n matrix whose rows store theta, d, a, alpha, and the joint type of each link.");
    dqserialmanipulatordh_py.def(
        py::init<MatrixXd>(),
        py::arg("dh_matrix"),
        "Constructs a serial manipulator from a standard DH matrix.");

    dqserialmanipulatordh_py.def(
        "get_thetas",
        &DQ_SerialManipulatorDH::get_thetas,
        "Returns the theta row of the stored DH matrix.");
    dqserialmanipulatordh_py.def(
        "get_ds",
        &DQ_SerialManipulatorDH::get_ds,
        "Returns the d row of the stored DH matrix.");
    dqserialmanipulatordh_py.def(
        "get_as",
        &DQ_SerialManipulatorDH::get_as,
        "Returns the a row of the stored DH matrix.");
    dqserialmanipulatordh_py.def(
        "get_alphas",
        &DQ_SerialManipulatorDH::get_alphas,
        "Returns the alpha row of the stored DH matrix.");
    dqserialmanipulatordh_py.def(
        "get_types",
        &DQ_SerialManipulatorDH::get_types,
        "Returns the joint-type row of the stored DH matrix as encoded joint types.");

    dqserialmanipulatordh_py.def(
        "raw_pose_jacobian",
        (MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDH::raw_pose_jacobian,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the raw pose Jacobian under the standard DH convention up to the requested link.");
    dqserialmanipulatordh_py.def(
        "raw_fkm",
        (DQ (DQ_SerialManipulatorDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDH::raw_fkm,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the raw forward kinematics under the standard DH convention up to the requested link.");
    dqserialmanipulatordh_py.def(
        "raw_pose_jacobian_derivative",
        (MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_SerialManipulatorDH::raw_pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_ith_link"),
        "Computes the time derivative of the raw pose Jacobian under the standard DH convention up to the requested link.");
}
