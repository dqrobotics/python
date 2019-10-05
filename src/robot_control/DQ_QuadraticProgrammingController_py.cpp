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

void init_DQ_QuadraticProgrammingController_py(py::module& m)
{
    /*****************************************************
     *  DQ TaskspaceQuadraticProgrammingController
     * **************************************************/
    py::class_<DQ_QuadraticProgrammingController, DQ_KinematicConstrainedController> dqtaskspacequadraticprogrammingcontroller_py(m,"DQ_TaskspaceQuadraticProgrammingController");
    dqtaskspacequadraticprogrammingcontroller_py.def("compute_objective_function_symmetric_matrix", &DQ_QuadraticProgrammingController::compute_objective_function_symmetric_matrix, "Compute symmetric matrix.");
    dqtaskspacequadraticprogrammingcontroller_py.def("compute_objective_function_linear_component", &DQ_QuadraticProgrammingController::compute_objective_function_linear_component, "Compute the objective function.");
    dqtaskspacequadraticprogrammingcontroller_py.def("compute_setpoint_control_signal", &DQ_QuadraticProgrammingController::compute_setpoint_control_signal, "Compute the setpoint control signal.");
    dqtaskspacequadraticprogrammingcontroller_py.def("compute_tracking_control_signal", &DQ_QuadraticProgrammingController::compute_tracking_control_signal, "Compute the tracking control signal.");

}
