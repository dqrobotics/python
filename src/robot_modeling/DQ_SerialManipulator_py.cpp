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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#include "../dqrobotics_module.h"

void init_DQ_SerialManipulator_py(py::module& m)
{
    /***************************************************
    *  DQ SerialManipulator
    * **************************************************/
    py::class_<
            DQ_SerialManipulator,
            std::shared_ptr<DQ_SerialManipulator>,
            DQ_Kinematics
            > dqserialmanipulator_py(m, "DQ_SerialManipulator");
    //dqserialmanipulator_py.def(py::init<int>());

    ///Methods
    //Concrete
    dqserialmanipulator_py.def("get_effector",                &DQ_SerialManipulator::get_effector,"Retrieves the effector.");
    dqserialmanipulator_py.def("set_effector",                &DQ_SerialManipulator::set_effector,"Sets the effector.");
    dqserialmanipulator_py.def("get_lower_q_limit",           &DQ_SerialManipulator::get_lower_q_limit,"Retrieves the lower limit for the joint values.");
    dqserialmanipulator_py.def("set_lower_q_limit",           &DQ_SerialManipulator::set_lower_q_limit,"Sets the lower limit for the joint values.");
    dqserialmanipulator_py.def("get_lower_q_dot_limit",       &DQ_SerialManipulator::get_lower_q_dot_limit,"Retrieves the lower limit for the joint velocities.");
    dqserialmanipulator_py.def("set_lower_q_dot_limit",       &DQ_SerialManipulator::set_lower_q_dot_limit,"Sets the lower limit for the joint velocities.");
    dqserialmanipulator_py.def("get_upper_q_limit",           &DQ_SerialManipulator::get_upper_q_limit,"Retrieves the upper limit for the joint values.");
    dqserialmanipulator_py.def("set_upper_q_limit",           &DQ_SerialManipulator::set_upper_q_limit,"Sets the upper limit for the joint values.");
    dqserialmanipulator_py.def("get_upper_q_dot_limit",       &DQ_SerialManipulator::get_upper_q_dot_limit,"Retrieves the upper limit for the joint velocities.");
    dqserialmanipulator_py.def("set_upper_q_dot_limit",       &DQ_SerialManipulator::set_upper_q_dot_limit,"Sets the upper limit for the joint velocities.");

    //Virtual
    dqserialmanipulator_py.def("raw_fkm",                     (DQ (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::raw_fkm,"Gets the raw fkm.");
    dqserialmanipulator_py.def("raw_pose_jacobian",           (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::raw_pose_jacobian,"Returns the pose Jacobian without base or effector transformation");
    dqserialmanipulator_py.def("raw_pose_jacobian_derivative",(MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const VectorXd&) const)
                                                               &DQ_SerialManipulator::raw_pose_jacobian_derivative,
                                                               "Returns the pose Jacobian derivative without base or effector transformation");

    //Overrides from DQ_Kinematics
    dqserialmanipulator_py.def("fkm",                         (DQ (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::fkm,"Gets the fkm.");
    dqserialmanipulator_py.def("fkm",                         (DQ (DQ_SerialManipulator::*)(const VectorXd&,const int&) const)&DQ_SerialManipulator::fkm,"Gets the fkm.");

    dqserialmanipulator_py.def("get_dim_configuration_space", &DQ_SerialManipulator::get_dim_configuration_space,"Retrieves the number of links.");

    dqserialmanipulator_py.def("pose_jacobian",               (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const int&) const)&DQ_SerialManipulator::pose_jacobian,"Returns the pose Jacobian");
    dqserialmanipulator_py.def("pose_jacobian",               (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::pose_jacobian,"Returns the pose Jacobian");
    dqserialmanipulator_py.def("pose_jacobian_derivative",    (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const VectorXd&, const int&) const)
                                                               &DQ_SerialManipulator::pose_jacobian_derivative,"Returns the pose Jacobian derivative");
    dqserialmanipulator_py.def("pose_jacobian_derivative",    (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const VectorXd&) const)
                                                               &DQ_SerialManipulator::pose_jacobian_derivative,"Returns the pose Jacobian derivative");
}
