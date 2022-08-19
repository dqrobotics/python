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

class DQ_KinematicControllerPub : public DQ_KinematicController
{
public:
    using DQ_KinematicController::_get_robot;
};

void init_DQ_KinematicController_py(py::module& m)
{
    /*****************************************************
     *  DQ KinematicController
     * **************************************************/
    py::class_<DQ_KinematicController> kc_py(m,"DQ_KinematicController");
    kc_py.def("get_control_objective"       ,&DQ_KinematicController::get_control_objective,"Gets the control objective");
    kc_py.def("get_jacobian"                ,&DQ_KinematicController::get_jacobian,"Gets the Jacobian");
    kc_py.def("get_last_error_signal"       ,&DQ_KinematicController::get_last_error_signal, "Gets the last error signal");
    kc_py.def("get_task_variable"           ,&DQ_KinematicController::get_task_variable, "Gets the task variable");
    kc_py.def("is_set"                      ,&DQ_KinematicController::is_set,"Checks if the controller's objective has been set");
    kc_py.def("system_reached_stable_region",&DQ_KinematicController::system_reached_stable_region,"Checks if the controller has stabilized");
    kc_py.def("set_control_objective"       ,&DQ_KinematicController::set_control_objective,"Sets the control objective");
    kc_py.def("set_gain"                    ,&DQ_KinematicController::set_gain,"Sets the controller gain");
    kc_py.def("get_gain"                    ,&DQ_KinematicController::get_gain,"Gets the controller gain");
    kc_py.def("set_stability_threshold"     ,&DQ_KinematicController::set_stability_threshold,"Sets the stability threshold");
    kc_py.def("set_damping"                 ,&DQ_KinematicController::set_damping, "Sets the damping.");
    kc_py.def("get_damping"                 ,&DQ_KinematicController::get_damping, "Gets the damping.");
    kc_py.def("set_primitive_to_effector"   ,&DQ_KinematicController::set_primitive_to_effector, "Sets the effector primitive");
    kc_py.def("set_target_primitive"        ,&DQ_KinematicController::set_target_primitive, "Sets the target primitive");
    kc_py.def("set_stability_counter_max"   ,&DQ_KinematicController::set_stability_counter_max, "Sets the maximum of the stability counter");
    kc_py.def("reset_stability_counter"     ,&DQ_KinematicController::reset_stability_counter, "Resets the stability counter");
    kc_py.def("_get_robot",&DQ_KinematicControllerPub::_get_robot , "Gets the robot");


}
