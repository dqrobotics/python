#include "../../dqrobotics_module.h"

void init_DQ_VrepRobot_py(py::module& m)
{
    /*****************************************************
     *  VrepRobot
     * **************************************************/
    py::class_<DQ_VrepRobot> dqvreprobot_py(m,"DQ_VrepRobot");
    dqvreprobot_py.def("send_q_to_vrep", &DQ_VrepRobot::send_q_to_vrep, "Get joint values from vrep.");
    dqvreprobot_py.def("get_q_from_vrep", &DQ_VrepRobot::get_q_from_vrep, "Send joint values to vrep.");

}
