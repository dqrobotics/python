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

class DQ_QuadraticProgrammingControllerPy : public DQ_QuadraticProgrammingController
{
protected:
    /* Inherit the constructors */
    using DQ_QuadraticProgrammingController::DQ_QuadraticProgrammingController;
public:
    DQ_QuadraticProgrammingControllerPy()=delete;
    DQ_QuadraticProgrammingControllerPy(const std::shared_ptr<DQ_Kinematics>& r,
                                        const std::shared_ptr<DQ_QuadraticProgrammingSolver>& s):
        DQ_QuadraticProgrammingController(r,s)
    {

    }

    //virtual VectorXd compute_setpoint_control_signal(const VectorXd&q, const VectorXd& task_reference) override;
    //virtual VectorXd compute_tracking_control_signal(const VectorXd&q, const VectorXd& task_reference, const VectorXd& feed_forward) override;

    /* Trampoline (need one for each virtual function) */
    MatrixXd compute_objective_function_symmetric_matrix(const MatrixXd& J, const VectorXd& task_error) override
    {
        PYBIND11_OVERLOAD_PURE(
                    MatrixXd,                       /* Return type */
                    DQ_QuadraticProgrammingController,  /* Parent class */
                    compute_objective_function_symmetric_matrix,        /* Name of function in C++ (must match Python name) */
                    J, task_error            /* Argument(s) */
                    )
    }

    VectorXd compute_objective_function_linear_component(const MatrixXd& J, const VectorXd& task_error) override
    {
        PYBIND11_OVERLOAD_PURE(
                    VectorXd,                       /* Return type */
                    DQ_QuadraticProgrammingController,  /* Parent class */
                    compute_objective_function_linear_component,        /* Name of function in C++ (must match Python name) */
                    J, task_error            /* Argument(s) */
                    )
    }
};


void init_DQ_QuadraticProgrammingController_py(py::module& m)
{
    /*****************************************************
     *  DQ TaskspaceQuadraticProgrammingController
     * **************************************************/
    py::class_<DQ_QuadraticProgrammingController, DQ_KinematicConstrainedController, DQ_QuadraticProgrammingControllerPy> qpcpy(m,"DQ_QuadraticProgrammingController");
    qpcpy.def(py::init<
              const std::shared_ptr<DQ_Kinematics>&,
              const std::shared_ptr<DQ_QuadraticProgrammingSolver>&
              >());
    qpcpy.def("compute_objective_function_symmetric_matrix", &DQ_QuadraticProgrammingController::compute_objective_function_symmetric_matrix, "Compute symmetric matrix.");
    qpcpy.def("compute_objective_function_linear_component", &DQ_QuadraticProgrammingController::compute_objective_function_linear_component, "Compute the objective function.");
    qpcpy.def("compute_setpoint_control_signal", &DQ_QuadraticProgrammingController::compute_setpoint_control_signal, "Compute the setpoint control signal.");
    qpcpy.def("compute_tracking_control_signal", &DQ_QuadraticProgrammingController::compute_tracking_control_signal, "Compute the tracking control signal.");
}
