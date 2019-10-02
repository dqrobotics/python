#include "../dqrobotics_module.h"

void init_DQ_WholeBody_py(py::module& m)
{
    /*****************************************************
     *  DQ WholeBody
     * **************************************************/
    py::class_<DQ_WholeBody,DQ_Kinematics> dqwholebody_py(m,"DQ_WholeBody");
    dqwholebody_py.def(py::init<std::shared_ptr<DQ_Kinematics>>());
    dqwholebody_py.def("add",&DQ_WholeBody::add,"Adds a DQ_Kinematics pointer to the kinematic chain.");
    dqwholebody_py.def("fkm",(DQ (DQ_WholeBody::*)(const VectorXd&) const)&DQ_WholeBody::fkm,"Gets the fkm.");
    dqwholebody_py.def("fkm",(DQ (DQ_WholeBody::*)(const VectorXd&,const int&) const)&DQ_WholeBody::fkm,"Gets the fkm.");
    dqwholebody_py.def("get_dim_configuration_space",&DQ_WholeBody::get_dim_configuration_space,"Gets the dimention of the configuration space");
    dqwholebody_py.def("get_chain",&DQ_WholeBody::get_chain, "Returns the DQ_Kinematics at a given index of the chain");
    dqwholebody_py.def("get_chain_as_serial_manipulator",&DQ_WholeBody::get_chain_as_serial_manipulator, "Returns the DQ_SerialManipulator at a given index of the chain");
    dqwholebody_py.def("get_chain_as_holonomic_base",&DQ_WholeBody::get_chain_as_holonomic_base, "Returns the DQ_HolonomicBase at a given index of the chain");
    dqwholebody_py.def("pose_jacobian",&DQ_WholeBody::pose_jacobian,"Returns the combined pose Jacobian");

}
