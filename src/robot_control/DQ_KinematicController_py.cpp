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

class DQ_KinematicControllerPub : public DQ_KinematicController
{
public:
    using DQ_KinematicController::_get_robot;
};

/**
 * @brief Binds `DQ_KinematicController`, an abstract class that defines an
 * interface to implement kinematic controllers for robots described by
 * DQ_Kinematics, to the Python module @p m.
 */
void init_DQ_KinematicController_py(py::module& m)
{
    /*****************************************************
     *  DQ KinematicController
     * **************************************************/
    py::class_<DQ_KinematicController> kc_py(
        m,
        "DQ_KinematicController",
        "Abstract class that defines an interface to implement kinematic controllers for robots described by DQ_Kinematics.");
    kc_py.def("get_control_objective",
              &DQ_KinematicController::get_control_objective,
              "Returns the current control objective.");
    kc_py.def("get_jacobian",
              &DQ_KinematicController::get_jacobian,
              py::arg("q"),
              "Returns the task Jacobian associated with the current control objective.");
    kc_py.def("get_last_error_signal",
              &DQ_KinematicController::get_last_error_signal,
              "Returns the last task-space error signal computed by the controller.");
    kc_py.def("get_task_variable",
              &DQ_KinematicController::get_task_variable,
              py::arg("q"),
              "Returns the current task variable associated with the control objective.");
    kc_py.def("is_set",
              &DQ_KinematicController::is_set,
              "Verifies whether a control objective has been selected.");
    kc_py.def("system_reached_stable_region",
              &DQ_KinematicController::system_reached_stable_region,
              "Indicates whether the closed-loop system has reached a stable region.");
    kc_py.def("set_control_objective",
              &DQ_KinematicController::set_control_objective,
              py::arg("control_objective"),
              "Sets the control objective and resizes the internally stored error vector to match the selected task variable.");
    kc_py.def("set_gain",
              &DQ_KinematicController::set_gain,
              py::arg("gain"),
              "Sets the controller gain.");
    kc_py.def("get_gain",
              &DQ_KinematicController::get_gain,
              "Returns the controller gain.");
    kc_py.def("set_stability_threshold",
              &DQ_KinematicController::set_stability_threshold,
              py::arg("threshold"),
              "Sets the threshold used to detect convergence to a stable region.");
    kc_py.def("set_damping",
              &DQ_KinematicController::set_damping,
              py::arg("damping"),
              "Sets the isotropic damping used by singularity-robust controllers.");
    kc_py.def("get_damping",
              &DQ_KinematicController::get_damping,
              "Returns the isotropic damping coefficient.");
    kc_py.def("set_primitive_to_effector",
              &DQ_KinematicController::set_primitive_to_effector,
              py::arg("primitive"),
              "Attaches a primitive to the end-effector for primitive-based objectives.");
    kc_py.def("set_target_primitive",
              &DQ_KinematicController::set_target_primitive,
              py::arg("primitive"),
              "Sets the target primitive for primitive-based convergence tasks.");
    kc_py.def("set_stability_counter_max",
              &DQ_KinematicController::set_stability_counter_max,
              py::arg("max"),
              "Sets the number of consecutive stable iterations required to declare convergence.");
    kc_py.def("reset_stability_counter",
              &DQ_KinematicController::reset_stability_counter,
              "Resets the stability counter and clears the stable-region flag.");
    kc_py.def("_get_robot",
              &DQ_KinematicControllerPub::_get_robot,
              "Returns the stored shared pointer to the associated robot model.");


}
