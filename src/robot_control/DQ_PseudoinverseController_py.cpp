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
 * @brief Binds `DQ_PseudoinverseController`, which implements a kinematic
 * control law based on the Jacobian pseudoinverse and an Euclidean task-space
 * error, to the Python module @p m.
 */
void init_DQ_PseudoinverseController_py(py::module& m)
{
    /*****************************************************
     *  DQ TaskSpacePseudoInverseController
     * **************************************************/
    py::class_<DQ_PseudoinverseController, DQ_KinematicController> dqpseudoinversecontroller_py(
        m,
        "DQ_PseudoinverseController",
        "Implements a kinematic control law based on the Jacobian pseudoinverse and an Euclidean task-space error.");
    dqpseudoinversecontroller_py.def(py::init<
                                     const std::shared_ptr<DQ_Kinematics>&
                                     >(),
                                     py::arg("robot"),
                                     "Constructs a controller from a shared robot pointer.");
    dqpseudoinversecontroller_py.def("compute_setpoint_control_signal",
                                     &DQ_PseudoinverseController::compute_setpoint_control_signal,
                                     py::arg("q"),
                                     py::arg("task_reference"),
                                     "Computes the reference joint velocities that drive the task-space error to zero.");
    dqpseudoinversecontroller_py.def("compute_tracking_control_signal",
                                     &DQ_PseudoinverseController::compute_tracking_control_signal,
                                     py::arg("q"),
                                     py::arg("task_reference"),
                                     py::arg("feed_forward"),
                                     "Computes the reference joint velocities for a time-varying task-space reference.");
}
