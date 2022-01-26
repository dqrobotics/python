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
- Juan Jose Quiroz Omana -  juanjqo@g.ecc.u-tokyo.ac.jp
*/

#include "../dqrobotics_module.h"

void init_DQ_SerialManipulatorMDH_py(py::module& m)
{
    /***************************************************
    *  DQ SerialManipulatorMDH
    * **************************************************/
    py::class_<DQ_SerialManipulatorMDH,DQ_SerialManipulator> dqserialmanipulatormdh_py(m, "DQ_SerialManipulatorMDH");
    dqserialmanipulatormdh_py.def(py::init<MatrixXd>());

    ///Methods
    //Concrete
    dqserialmanipulatormdh_py.def("get_thetas",  &DQ_SerialManipulatorMDH::get_thetas, "Retrieves the vector of thetas.");
    dqserialmanipulatormdh_py.def("get_ds",      &DQ_SerialManipulatorMDH::get_ds,     "Retrieves the vector of ds.");
    dqserialmanipulatormdh_py.def("get_as",      &DQ_SerialManipulatorMDH::get_as,     "Retrieves the vector of as.");
    dqserialmanipulatormdh_py.def("get_alphas",  &DQ_SerialManipulatorMDH::get_alphas, "Retrieves the vector of alphas.");
    dqserialmanipulatormdh_py.def("get_types",   &DQ_SerialManipulatorMDH::get_types,  "Retrieves the vector of types.");

    dqserialmanipulatormdh_py.def("pose_jacobian_derivative", (MatrixXd (DQ_SerialManipulatorMDH::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_SerialManipulatorMDH::pose_jacobian_derivative, "Returns the pose Jacobian derivative");
    dqserialmanipulatormdh_py.def("pose_jacobian_derivative", (MatrixXd (DQ_SerialManipulatorMDH::*)(const VectorXd&, const VectorXd&) const)&DQ_SerialManipulatorMDH::pose_jacobian_derivative,             "Returns the pose Jacobian derivative");

    //Overrides from DQ_SerialManipulator
    dqserialmanipulatormdh_py.def("raw_pose_jacobian",  (MatrixXd (DQ_SerialManipulatorMDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorMDH::raw_pose_jacobian, "Retrieves the raw pose Jacobian.");
    dqserialmanipulatormdh_py.def("raw_fkm",            (DQ (DQ_SerialManipulatorMDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorMDH::raw_fkm,                 "Retrieves the raw FKM.");
}
