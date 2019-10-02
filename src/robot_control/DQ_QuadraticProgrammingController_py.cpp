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
