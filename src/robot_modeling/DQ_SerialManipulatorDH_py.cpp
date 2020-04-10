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

void init_DQ_SerialManipulatorDH_py(py::module& m)
{
    /***************************************************
    *  DQ SerialManipulatorDH
    * **************************************************/
    py::class_<DQ_SerialManipulatorDH,DQ_SerialManipulator,DQ_Kinematics> dqserialmanipulatordh_py(m, "DQ_SerialManipulatorDH");
    dqserialmanipulatordh_py.def(py::init<MatrixXd, std::string>());

    dqserialmanipulatordh_py.def("type",                        &DQ_SerialManipulatorDH::type,"Retrieves the vector of types.");

    ///Methods (From DQ_SerialManipulator)
    dqserialmanipulatordh_py.def("getDHMatrix",                 &DQ_SerialManipulatorDH::getDHMatrix,"Gets the DH matrix.");
    dqserialmanipulatordh_py.def("theta",                       &DQ_SerialManipulatorDH::theta,"Retrieves the vector of thetas.");
    dqserialmanipulatordh_py.def("d",                           &DQ_SerialManipulatorDH::d,"Retrieves the vector d.");
    dqserialmanipulatordh_py.def("a",                           &DQ_SerialManipulatorDH::a,"Retrieves the vector a.");
    dqserialmanipulatordh_py.def("alpha",                       &DQ_SerialManipulatorDH::alpha,"Retrieves the vector of alphas.");
    dqserialmanipulatordh_py.def("get_dim_configuration_space", &DQ_SerialManipulatorDH::get_dim_configuration_space,"Retrieves the number of links.");
    dqserialmanipulatordh_py.def("convention",                  &DQ_SerialManipulatorDH::convention,"Retrieves the DH convention.");
    dqserialmanipulatordh_py.def("base_frame",                  &DQ_SerialManipulatorDH::get_base_frame,"Retrieves the base.");
    dqserialmanipulatordh_py.def("reference_frame",             &DQ_SerialManipulatorDH::get_reference_frame,"Retrieves the reference frame");
    dqserialmanipulatordh_py.def("effector",                    &DQ_SerialManipulatorDH::effector,"Retrieves the effector.");
    dqserialmanipulatordh_py.def("set_base_frame",              &DQ_SerialManipulatorDH::set_base_frame,"Sets the base.");
    dqserialmanipulatordh_py.def("set_reference_frame",         &DQ_SerialManipulatorDH::set_reference_frame,"Sets the reference frame");
    dqserialmanipulatordh_py.def("set_effector",                &DQ_SerialManipulatorDH::set_effector,"Sets the effector.");

    dqserialmanipulatordh_py.def("raw_fkm",                     (DQ (DQ_SerialManipulatorDH::*)(const VectorXd&) const)&DQ_SerialManipulatorDH::raw_fkm,"Gets the raw fkm.");
    dqserialmanipulatordh_py.def("raw_fkm",                     (DQ (DQ_SerialManipulatorDH::*)(const VectorXd&,const int&) const)&DQ_SerialManipulatorDH::raw_fkm,"Gets the raw fkm.");

    dqserialmanipulatordh_py.def("fkm",                         (DQ (DQ_SerialManipulatorDH::*)(const VectorXd&) const)&DQ_SerialManipulatorDH::fkm,"Gets the fkm.");
    dqserialmanipulatordh_py.def("fkm",                         (DQ (DQ_SerialManipulatorDH::*)(const VectorXd&,const int&) const)&DQ_SerialManipulatorDH::fkm,"Gets the fkm.");

    dqserialmanipulatordh_py.def("raw_pose_jacobian",           (MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDH::raw_pose_jacobian,"Returns the pose Jacobian without base or effector transformation");
    dqserialmanipulatordh_py.def("raw_pose_jacobian",           (MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&) const)&DQ_SerialManipulatorDH::raw_pose_jacobian,"Returns the pose Jacobian without base or effector transformation");

    dqserialmanipulatordh_py.def("pose_jacobian",               (MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDH::pose_jacobian,"Returns the pose Jacobian");
    dqserialmanipulatordh_py.def("pose_jacobian",               (MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&) const)&DQ_SerialManipulatorDH::pose_jacobian,"Returns the pose Jacobian");

    dqserialmanipulatordh_py.def("pose_jacobian_derivative",    (MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&, const VectorXd&, const int&) const)&DQ_SerialManipulatorDH::pose_jacobian_derivative,"Returns the derivative of the pose Jacobian");
    dqserialmanipulatordh_py.def("pose_jacobian_derivative",    (MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&, const VectorXd&) const)&DQ_SerialManipulatorDH::pose_jacobian_derivative,"Returns the derivative of the pose Jacobian");
}
