/**
(C) Copyright 2019 DQ Robotics Developers

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

void init_DQ_WholeBody_py(py::module& m)
{
    /*****************************************************
     *  DQ WholeBody
     * **************************************************/
    py::class_<
            DQ_WholeBody,
            std::shared_ptr<DQ_WholeBody>,
            DQ_Kinematics
            > dqwholebody_py(m,"DQ_WholeBody");
    dqwholebody_py.def(py::init<std::shared_ptr<DQ_Kinematics>>());
    dqwholebody_py.def("add",&DQ_WholeBody::add,"Adds a DQ_Kinematics pointer to the kinematic chain.");
    dqwholebody_py.def("fkm",(DQ (DQ_WholeBody::*)(const VectorXd&) const)&DQ_WholeBody::fkm,"Gets the fkm.");
    dqwholebody_py.def("fkm",(DQ (DQ_WholeBody::*)(const VectorXd&,const int&) const)&DQ_WholeBody::fkm,"Gets the fkm.");
    dqwholebody_py.def("get_dim_configuration_space",&DQ_WholeBody::get_dim_configuration_space,"Gets the dimention of the configuration space");
    dqwholebody_py.def("get_chain",&DQ_WholeBody::get_chain, "Returns the DQ_Kinematics at a given index of the chain");
    dqwholebody_py.def("get_chain_as_serial_manipulator_dh",&DQ_WholeBody::get_chain_as_serial_manipulator_dh, "Returns the DQ_SerialManipulatorDH at a given index of the chain");
    dqwholebody_py.def("get_chain_as_holonomic_base",&DQ_WholeBody::get_chain_as_holonomic_base, "Returns the DQ_HolonomicBase at a given index of the chain");
    dqwholebody_py.def("pose_jacobian",(MatrixXd (DQ_WholeBody::*)(const VectorXd&, const int&) const)&DQ_WholeBody::pose_jacobian,"Returns the pose Jacobian");
    dqwholebody_py.def("pose_jacobian",(MatrixXd (DQ_WholeBody::*)(const VectorXd&) const)&DQ_WholeBody::pose_jacobian,"Returns the pose Jacobian");
}
