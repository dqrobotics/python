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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include "../dqrobotics_module.h"

void init_DQ_KinematicController_py(py::module& m)
{
    /*****************************************************
     *  DQ KinematicController
     * **************************************************/
    py::class_<DQ_KinematicController> dqkinematiccontroller_py(m,"DQ_KinematicController");
    dqkinematiccontroller_py.def("get_control_objective"     ,&DQ_KinematicController::get_control_objective,"Gets the control objective");
    dqkinematiccontroller_py.def("get_jacobian"              ,&DQ_KinematicController::get_jacobian,"Gets the Jacobian");
    dqkinematiccontroller_py.def("get_last_error_signal"     ,&DQ_KinematicController::get_last_error_signal, "Gets the last error signal");
    dqkinematiccontroller_py.def("get_task_variable"         ,&DQ_KinematicController::get_task_variable, "Gets the task variable");
    dqkinematiccontroller_py.def("is_set"                    ,&DQ_KinematicController::is_set,"Checks if the controller's objective has been set");
    dqkinematiccontroller_py.def("is_stable"                 ,&DQ_KinematicController::is_stable,"Checks if the controller has stabilized");
    dqkinematiccontroller_py.def("set_control_objective"     ,&DQ_KinematicController::set_control_objective,"Sets the control objective");
    dqkinematiccontroller_py.def("set_gain"                  ,&DQ_KinematicController::set_gain,"Sets the controller gain");
    dqkinematiccontroller_py.def("set_stability_threshold"   ,&DQ_KinematicController::set_stability_threshold,"Sets the stability threshold");
    dqkinematiccontroller_py.def("set_damping"               ,&DQ_KinematicController::set_damping, "Sets the damping.");
    dqkinematiccontroller_py.def("set_primitive_to_effector" ,&DQ_KinematicController::set_primitive_to_effector, "Sets the effector primitive");
}
