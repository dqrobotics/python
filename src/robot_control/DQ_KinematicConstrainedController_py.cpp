#include "../dqrobotics_module.h"

void init_DQ_KinematicConstrainedController_py(py::module& m)
{
    /*****************************************************
     *  DQ KinematicConstrainedController
     * **************************************************/
    py::class_<DQ_KinematicConstrainedController, DQ_KinematicController> dqkinematicconstrainedcontroller_py(m,"DQ_KinematicConstrainedController");
    dqkinematicconstrainedcontroller_py.def("set_equality_constraint", &DQ_KinematicConstrainedController::set_equality_constraint,  "Sets equality constraints.");
    dqkinematicconstrainedcontroller_py.def("set_inequality_constraint", &DQ_KinematicConstrainedController::set_inequality_constraint,  "Sets inequality constraints.");

}
