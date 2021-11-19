/**
(C) Copyright 2020 DQ Robotics Developers

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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include "../dqrobotics_module.h"

void init_DQ_SerialWholeBody_py(py::module& m)
{
    /*****************************************************
     *  DQ WholeBody
     * **************************************************/
    py::class_<DQ_SerialWholeBody,DQ_Kinematics> dqserialwholebody_py(m,"DQ_SerialWholeBody");
    dqserialwholebody_py.def(py::init<std::shared_ptr<DQ_Kinematics>>());
    dqserialwholebody_py.def("add",&DQ_SerialWholeBody::add,"Adds a DQ_Kinematics pointer to the kinematic chain.");
    dqserialwholebody_py.def("fkm",(DQ (DQ_SerialWholeBody::*)(const VectorXd&) const)&DQ_SerialWholeBody::fkm,"Gets the fkm.");
    dqserialwholebody_py.def("fkm",(DQ (DQ_SerialWholeBody::*)(const VectorXd&,const int&) const)&DQ_SerialWholeBody::fkm,"Gets the fkm.");
    dqserialwholebody_py.def("get_dim_configuration_space",&DQ_SerialWholeBody::get_dim_configuration_space,"Gets the dimention of the configuration space");
    dqserialwholebody_py.def("get_chain",&DQ_SerialWholeBody::get_chain, "Returns the DQ_Kinematics at a given index of the chain");
    dqserialwholebody_py.def("get_chain_as_serial_manipulator_dh",&DQ_SerialWholeBody::get_chain_as_serial_manipulator_dh, "Returns the DQ_SerialManipulatorDH at a given index of the chain");
    dqserialwholebody_py.def("get_chain_as_holonomic_base",&DQ_SerialWholeBody::get_chain_as_holonomic_base, "Returns the DQ_HolonomicBase at a given index of the chain");
    dqserialwholebody_py.def("pose_jacobian",(MatrixXd (DQ_SerialWholeBody::*)(const VectorXd&, const int&) const)&DQ_SerialWholeBody::pose_jacobian,"Returns the pose Jacobian");
    dqserialwholebody_py.def("pose_jacobian",(MatrixXd (DQ_SerialWholeBody::*)(const VectorXd&) const)&DQ_SerialWholeBody::pose_jacobian,"Returns the pose Jacobian");
}
