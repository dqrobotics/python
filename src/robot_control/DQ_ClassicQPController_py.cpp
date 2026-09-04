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
 * @brief Binds `DQ_ClassicQPController`, which implements the classic
 * quadratic-programming kinematic controller based on task-space variables, to
 * the Python module @p m.
 */
void init_DQ_ClassicQPController_py(py::module& m)
{
    /*****************************************************
     *  DQ ClassicQPController
     * **************************************************/
    py::class_<DQ_ClassicQPController, DQ_QuadraticProgrammingController> dq_classicqpcontroller_py(
        m,
        "DQ_ClassicQPController",
        "Implements the classic quadratic-programming kinematic controller based on task-space variables.");
    dq_classicqpcontroller_py.def(py::init<
                                  const std::shared_ptr<DQ_Kinematics>&,
                                  const std::shared_ptr<DQ_QuadraticProgrammingSolver>&
                                  >(),
                                  py::arg("robot"),
                                  py::arg("solver"),
                                  "Constructs a classic QP controller from shared pointers.");
    dq_classicqpcontroller_py.def("compute_objective_function_symmetric_matrix",
                                  &DQ_ClassicQPController::compute_objective_function_symmetric_matrix,
                                  py::arg("J"),
                                  py::arg("task_error"),
                                  "Computes the symmetric matrix H used in the quadratic objective.");
    dq_classicqpcontroller_py.def("compute_objective_function_linear_component",
                                  &DQ_ClassicQPController::compute_objective_function_linear_component,
                                  py::arg("J"),
                                  py::arg("task_error"),
                                  "Computes the linear vector f used in the quadratic objective.");
}
