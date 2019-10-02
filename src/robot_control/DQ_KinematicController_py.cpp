#include "../dqrobotics_module.h"

void init_DQ_KinematicController_py(py::module& m)
{
    /*****************************************************
     *  DQ KinematicController
     * **************************************************/
    py::class_<DQ_KinematicController> dqkinematiccontroller_py(m,"DQ_KinematicController");
    dqkinematiccontroller_py.def("get_control_objective"  ,&DQ_KinematicController::get_control_objective,"Gets the control objective");
    dqkinematiccontroller_py.def("get_jacobian"           ,&DQ_KinematicController::get_jacobian,"Gets the Jacobian");
    dqkinematiccontroller_py.def("get_last_error_signal"  ,&DQ_KinematicController::get_last_error_signal, "Gets the last error signal");
    dqkinematiccontroller_py.def("is_set"                 ,&DQ_KinematicController::is_set,"Checks if the controller's objective has been set");
    dqkinematiccontroller_py.def("is_stable"              ,&DQ_KinematicController::is_stable,"Checks if the controller has stabilized");
    dqkinematiccontroller_py.def("set_control_objective"  ,&DQ_KinematicController::set_control_objective,"Sets the control objective");
    dqkinematiccontroller_py.def("set_gain"               ,&DQ_KinematicController::set_gain,"Sets the controller gain");
    dqkinematiccontroller_py.def("set_stability_threshold",&DQ_KinematicController::set_stability_threshold,"Sets the stability threshold");
    dqkinematiccontroller_py.def("set_damping"            ,&DQ_KinematicController::set_damping, "Sets the damping.");
}
