#include "../dqrobotics_module.h"

void init_DQ_HolonomicBase_py(py::module& m)
{
    /*****************************************************
     *  DQ HolonomicBase
     * **************************************************/
    py::class_<DQ_HolonomicBase,DQ_MobileBase> dqholonomicbase_py(m,"DQ_HolonomicBase");
    dqholonomicbase_py.def(py::init());
    dqholonomicbase_py.def("fkm",                        &DQ_HolonomicBase::fkm,"Returns the base's fkm");
    dqholonomicbase_py.def("pose_jacobian",              &DQ_HolonomicBase::pose_jacobian,"Returns the base's Jacobian");
    dqholonomicbase_py.def("get_dim_configuration_space",&DQ_HolonomicBase::get_dim_configuration_space,"Returns the size of the configuration space");
    dqholonomicbase_py.def("raw_fkm",                    &DQ_HolonomicBase::raw_fkm,"Returns the raw fkm");
    dqholonomicbase_py.def("raw_pose_jacobian",          &DQ_HolonomicBase::raw_pose_jacobian,"Return the raw ose Jacobian");
}
