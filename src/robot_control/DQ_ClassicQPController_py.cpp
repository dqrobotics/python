#include "../dqrobotics_module.h"

void init_DQ_ClassicQPController_py(py::module& m)
{
    /*****************************************************
     *  DQ ClassicQPController
     * **************************************************/
    py::class_<DQ_ClassicQPController, DQ_TaskspaceQuadraticProgrammingController> dq_classicqpcontroller_py(m,"DQ_ClassicQPController");
    dq_classicqpcontroller_py.def(py::init<DQ_Kinematics*, DQ_QuadraticProgrammingSolver*>());
    dq_classicqpcontroller_py.def("compute_objective_function_symmetric_matrix", &DQ_ClassicQPController::compute_objective_function_symmetric_matrix, "Compute symmetric matrix.");
    dq_classicqpcontroller_py.def("compute_objective_function_linear_component", &DQ_ClassicQPController::compute_objective_function_linear_component, "Compute the objective function.");
}
