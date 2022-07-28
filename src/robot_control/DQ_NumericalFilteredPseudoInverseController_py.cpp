/**
(C) Copyright 2022 DQ Robotics Developers

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
- Murilo M. Marinho (murilo@g.ecc.u-tokyo.ac.jp)
*/

#include "../dqrobotics_module.h"

void init_DQ_NumericalFilteredPseudoInverseController_py(py::module& m)
{
    /*****************************************************
     *  DQ TaskSpacePseudoInverseController
     * **************************************************/
    py::class_<DQ_NumericalFilteredPseudoinverseController, DQ_PseudoinverseController> nfpic(m,"DQ_NumericalFilteredPseudoInverseController");
    nfpic.def(py::init<DQ_Kinematics*>());
    nfpic.def("compute_setpoint_control_signal",&DQ_NumericalFilteredPseudoinverseController::compute_setpoint_control_signal,"Computes the setpoint control signal.");
    nfpic.def("compute_tracking_control_signal",&DQ_NumericalFilteredPseudoinverseController::compute_tracking_control_signal,"Computes the tracking control signal.");
    nfpic.def("set_maximum_numerical_filtered_damping",&DQ_NumericalFilteredPseudoinverseController::set_maximum_numerical_filtered_damping,"Sets the maximum numerical filtered damping.");
    nfpic.def("set_singular_region_size",&DQ_NumericalFilteredPseudoinverseController::set_singular_region_size,"Sets the singular region size.");
    nfpic.def("get_maximum_numerical_filtered_damping",&DQ_NumericalFilteredPseudoinverseController::get_maximum_numerical_filtered_damping,"Gets the maximum numerical filtered damping.");
    nfpic.def("get_singular_region_size",&DQ_NumericalFilteredPseudoinverseController::get_singular_region_size,"Gets the singular region size.");
    nfpic.def("get_last_filtered_damping",&DQ_NumericalFilteredPseudoinverseController::get_last_filtered_damping,"Gets the last filtered damping.");
    nfpic.def("get_last_jacobian_rank",&DQ_NumericalFilteredPseudoinverseController::get_last_jacobian_rank,"Gets the last Jacobian rank.");
    nfpic.def("get_last_jacobian_svd",&DQ_NumericalFilteredPseudoinverseController::get_last_jacobian_svd,"Gets the last Jacobian svd.");
}
