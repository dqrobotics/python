#include "../dqrobotics_module.h"

void init_DQ_MobileBase_py(py::module& m)
{
    /*****************************************************
     *  DQ MobileBase
     * **************************************************/
    py::class_<DQ_MobileBase,DQ_Kinematics> dqmobilebase_py(m,"DQ_MobileBase");
    dqmobilebase_py.def("set_frame_displacement", &DQ_MobileBase::set_frame_displacement,"Set the frame displacement");
    dqmobilebase_py.def("frame_displacement",     &DQ_MobileBase::frame_displacement,    "Get the frame displacement");
}
