/**
(C) Copyright 2021 DQ Robotics Developers

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
 * @brief Binds `DQ_SerialManipulatorDenso`, the concrete serial manipulator
 * that uses the DENSO kinematic convention, to the Python module @p m.
 */
void init_DQ_SerialManipulatorDenso_py(py::module& m)
{
    py::class_<
            DQ_SerialManipulatorDenso,
            std::shared_ptr<DQ_SerialManipulatorDenso>,
            DQ_SerialManipulator> dqserialmanipulatordh_py(
                m,
                "DQ_SerialManipulatorDenso",
                "Concrete serial manipulator that uses the DENSO kinematic convention. The constructor expects a 6 x n matrix whose rows store the convention parameters a, b, d, alpha, beta, and gamma for each link.");
    dqserialmanipulatordh_py.def(
        py::init<MatrixXd>(),
        py::arg("denso_matrix"),
        "Constructs a serial manipulator from a DENSO-parameter matrix.");

    dqserialmanipulatordh_py.def(
        "get_as",
        &DQ_SerialManipulatorDenso::get_as,
        "Returns the a row of the stored DENSO matrix.");
    dqserialmanipulatordh_py.def(
        "get_bs",
        &DQ_SerialManipulatorDenso::get_bs,
        "Returns the b row of the stored DENSO matrix.");
    dqserialmanipulatordh_py.def(
        "get_ds",
        &DQ_SerialManipulatorDenso::get_ds,
        "Returns the d row of the stored DENSO matrix.");
    dqserialmanipulatordh_py.def(
        "get_alphas",
        &DQ_SerialManipulatorDenso::get_alphas,
        "Returns the alpha row of the stored DENSO matrix.");
    dqserialmanipulatordh_py.def(
        "get_betas",
        &DQ_SerialManipulatorDenso::get_betas,
        "Returns the beta row of the stored DENSO matrix.");
    dqserialmanipulatordh_py.def(
        "get_thetas",
        &DQ_SerialManipulatorDenso::get_gammas,
        "Returns the gamma row of the stored DENSO matrix.");

    dqserialmanipulatordh_py.def(
        "raw_pose_jacobian",
        (MatrixXd (DQ_SerialManipulatorDenso::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDenso::raw_pose_jacobian,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the raw pose Jacobian under the DENSO convention up to the requested link.");
    dqserialmanipulatordh_py.def(
        "raw_fkm",
        (DQ (DQ_SerialManipulatorDenso::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDenso::raw_fkm,
        py::arg("q_vec"),
        py::arg("to_ith_link"),
        "Computes the raw forward kinematics under the DENSO convention up to the requested link.");
    dqserialmanipulatordh_py.def(
        "raw_pose_jacobian_derivative",
        (MatrixXd (DQ_SerialManipulatorDenso::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_SerialManipulatorDenso::raw_pose_jacobian_derivative,
        py::arg("q"),
        py::arg("q_dot"),
        py::arg("to_ith_link"),
        "Computes the time derivative of the raw pose Jacobian under the DENSO convention up to the requested link.");
}
