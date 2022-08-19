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

class DQ_NumericalFilteredPseudoinverseControllerPy : public DQ_NumericalFilteredPseudoinverseController
{
public:
    using DQ_NumericalFilteredPseudoinverseController::DQ_NumericalFilteredPseudoinverseController;
    DQ_NumericalFilteredPseudoinverseControllerPy()=delete;
    DQ_NumericalFilteredPseudoinverseControllerPy(const std::shared_ptr<DQ_Kinematics>& r):
        DQ_NumericalFilteredPseudoinverseController(r)
    {

    }

    /* Trampoline (need one for each virtual function) */
    VectorXd compute_setpoint_control_signal(const VectorXd &q, const VectorXd &task_reference) override
    {
        PYBIND11_OVERRIDE(
                    VectorXd,                       /* Return type */
                    DQ_NumericalFilteredPseudoinverseController,  /* Parent class */
                    compute_setpoint_control_signal,        /* Name of function in C++ (must match Python name) */
                    q,task_reference          /* Argument(s) */
                    );
    }

    VectorXd compute_tracking_control_signal(const VectorXd &q, const VectorXd &task_reference, const VectorXd &feed_forward) override
    {
        PYBIND11_OVERRIDE(
                    VectorXd,                       /* Return type */
                    DQ_NumericalFilteredPseudoinverseController,  /* Parent class */
                    compute_tracking_control_signal,        /* Name of function in C++ (must match Python name) */
                    q,task_reference,feed_forward           /* Argument(s) */
                    );
    }
};


void init_DQ_NumericalFilteredPseudoInverseController_py(py::module& m)
{
    /*****************************************************
     *  DQ TaskSpacePseudoInverseController
     * **************************************************/
    py::class_<
            DQ_NumericalFilteredPseudoinverseController,
            DQ_NumericalFilteredPseudoinverseControllerPy,
            DQ_PseudoinverseController
            > nfpic(m,"DQ_NumericalFilteredPseudoinverseController");
    nfpic.def(py::init<
              const std::shared_ptr<DQ_Kinematics>&
              >());
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
