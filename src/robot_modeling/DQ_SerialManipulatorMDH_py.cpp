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
- Juan Jose Quiroz Omana -  juanjqo@g.ecc.u-tokyo.ac.jp
*/

#include "../dqrobotics_module.h"

/**
 * @brief Binds `DQ_SerialManipulatorMDH`, the concrete serial manipulator
 * based on the modified Denavit-Hartenberg convention, to the Python module
 * @p m.
 */
void init_DQ_SerialManipulatorMDH_py(py::module& m)
{
    py::class_<
            DQ_SerialManipulatorMDH,
            std::shared_ptr<DQ_SerialManipulatorMDH>,
            DQ_SerialManipulator
            > dqserialmanipulatormdh_py(
                m,
                "DQ_SerialManipulatorMDH",
                "Concrete serial manipulator based on the modified Denavit-Hartenberg convention. The constructor expects a 5 x n matrix whose rows store theta, d, a, alpha, and the joint type of each link.");
    dqserialmanipulatormdh_py.def(
        py::init<MatrixXd>(),
        py::arg("mdh_matrix"),
        "Constructs a serial manipulator from a modified DH matrix.");

    dqserialmanipulatormdh_py.def(
        "get_thetas",
        &DQ_SerialManipulatorMDH::get_thetas,
        "Returns the theta row of the stored modified DH matrix.");
    dqserialmanipulatormdh_py.def(
        "get_ds",
        &DQ_SerialManipulatorMDH::get_ds,
        "Returns the d row of the stored modified DH matrix.");
    dqserialmanipulatormdh_py.def(
        "get_as",
        &DQ_SerialManipulatorMDH::get_as,
        "Returns the a row of the stored modified DH matrix.");
    dqserialmanipulatormdh_py.def(
        "get_alphas",
        &DQ_SerialManipulatorMDH::get_alphas,
        "Returns the alpha row of the stored modified DH matrix.");
    dqserialmanipulatormdh_py.def(
        "get_types",
        &DQ_SerialManipulatorMDH::get_types,
        "Returns the joint-type row of the stored modified DH matrix as encoded joint types.");

    dqserialmanipulatormdh_py.def(
        "raw_pose_jacobian",
        (MatrixXd (DQ_SerialManipulatorMDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorMDH::raw_pose_jacobian,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the raw pose Jacobian under the modified DH convention up to the requested link.");
    dqserialmanipulatormdh_py.def(
        "raw_fkm",
        (DQ (DQ_SerialManipulatorMDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorMDH::raw_fkm,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the raw forward kinematics under the modified DH convention up to the requested link.");
    dqserialmanipulatormdh_py.def(
        "raw_pose_jacobian_derivative",
        (MatrixXd (DQ_SerialManipulatorMDH::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_SerialManipulatorMDH::raw_pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_ith_link"),
        "Computes the time derivative of the raw pose Jacobian under the modified DH convention up to the requested link.");
}
