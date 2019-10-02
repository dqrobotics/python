#include "../dqrobotics_module.h"

void init_DQ_DifferentialDriveRobot_py(py::module& m)
{
    /*****************************************************
     *  DQ DifferentialDriveRobot
     * **************************************************/
    py::class_<DQ_DifferentialDriveRobot,DQ_HolonomicBase> dqdifferentialdriverobot_py(m,"DQ_DifferentialDriveRobot");
    dqdifferentialdriverobot_py.def(py::init<const double&, const double&>());
    dqdifferentialdriverobot_py.def("constraint_jacobian", &DQ_DifferentialDriveRobot::constraint_jacobian, "Returns the constraint Jacobian");
    dqdifferentialdriverobot_py.def("pose_jacobian",       &DQ_DifferentialDriveRobot::pose_jacobian,       "Returns the pose Jacobian");
}
