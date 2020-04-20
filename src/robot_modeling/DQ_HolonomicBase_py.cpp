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

void init_DQ_HolonomicBase_py(py::module& m)
{
    /*****************************************************
     *  DQ HolonomicBase
     * **************************************************/
    py::class_<DQ_HolonomicBase,DQ_MobileBase> dqholonomicbase_py(m,"DQ_HolonomicBase");
    dqholonomicbase_py.def(py::init());
    dqholonomicbase_py.def("fkm",(DQ (DQ_HolonomicBase::*)(const VectorXd&) const)&DQ_HolonomicBase::fkm,"Gets the fkm.");
    dqholonomicbase_py.def("fkm",(DQ (DQ_HolonomicBase::*)(const VectorXd&,const int&) const)&DQ_HolonomicBase::fkm,"Gets the fkm.");
    dqholonomicbase_py.def("pose_jacobian",              &DQ_HolonomicBase::pose_jacobian,"Returns the base's Jacobian");
    dqholonomicbase_py.def("get_dim_configuration_space",&DQ_HolonomicBase::get_dim_configuration_space,"Returns the size of the configuration space");
    dqholonomicbase_py.def("raw_fkm",                    &DQ_HolonomicBase::raw_fkm,"Returns the raw fkm");
    dqholonomicbase_py.def("raw_pose_jacobian",          &DQ_HolonomicBase::raw_pose_jacobian,"Return the raw ose Jacobian");
}
