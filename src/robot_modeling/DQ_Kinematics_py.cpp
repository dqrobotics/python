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

void init_DQ_Kinematics_py(py::module& m)
{
    /*****************************************************
     *  DQ Kinematics
     * **************************************************/
    py::class_<
            DQ_Kinematics,
            std::shared_ptr<DQ_Kinematics>
            > dqkinematics_py(m, "DQ_Kinematics");
    //dqkinematics_py.def(py::init<>());
    //dqkinematics_py.def("pose_jacobian",  (MatrixXd (DQ_Kinematics::*)(const VectorXd&) const)&DQ_Kinematics::pose_jacobian, "Returns the pose Jacobian");

    //Concrete
    dqkinematics_py.def("get_dim_configuration_space", &DQ_Kinematics::get_dim_configuration_space, "Returns the dimension of the configuration space");
    dqkinematics_py.def("get_reference_frame",         &DQ_Kinematics::get_reference_frame, "Returns the current reference frame");
    dqkinematics_py.def("set_reference_frame",         &DQ_Kinematics::set_reference_frame, "Sets the reference frame");
    dqkinematics_py.def("get_base_frame",              &DQ_Kinematics::get_base_frame, "Returns the current base frame");
    dqkinematics_py.def("set_base_frame",              &DQ_Kinematics::set_base_frame, "Sets the base frame");

    //Static
    dqkinematics_py.def_static("distance_jacobian",                &DQ_Kinematics::distance_jacobian,     "Returns the distance Jacobian");
    dqkinematics_py.def_static("translation_jacobian",             &DQ_Kinematics::translation_jacobian,  "Returns the translation Jacobian");
    dqkinematics_py.def_static("rotation_jacobian",                &DQ_Kinematics::rotation_jacobian,     "Returns the rotation Jacobian");
    dqkinematics_py.def_static("line_jacobian",                    &DQ_Kinematics::line_jacobian,         "Returns the line Jacobian");
    dqkinematics_py.def_static("plane_jacobian",                   &DQ_Kinematics::plane_jacobian,        "Returns the plane Jacobian");
    dqkinematics_py.def_static("distance_jacobian_derivative",     &DQ_Kinematics::distance_jacobian_derivative,    "Returns the distance Jacobian derivative");
    dqkinematics_py.def_static("translation_jacobian_derivative",  &DQ_Kinematics::translation_jacobian_derivative, "Returns the translation Jacobian derivative");
    dqkinematics_py.def_static("rotation_jacobian_derivative",     &DQ_Kinematics::rotation_jacobian_derivative,    "Returns the rotation Jacobian derivative");
    dqkinematics_py.def_static("line_jacobian_derivative",         &DQ_Kinematics::line_jacobian_derivative,        "Returns the line Jacobian derivative");
    dqkinematics_py.def_static("plane_jacobian_derivative",        &DQ_Kinematics::plane_jacobian_derivative,       "Returns the plane Jacobian derivative");
    dqkinematics_py.def_static("point_to_point_distance_jacobian", &DQ_Kinematics::point_to_point_distance_jacobian,"Returns the robot point to point distance Jacobian");
    dqkinematics_py.def_static("point_to_point_residual",          &DQ_Kinematics::point_to_point_residual,"Returns the robot point to point residual");
    dqkinematics_py.def_static("point_to_line_distance_jacobian",  &DQ_Kinematics::point_to_line_distance_jacobian,"Returns the robot point to line distance Jacobian");
    dqkinematics_py.def_static("point_to_line_residual",           &DQ_Kinematics::point_to_line_residual,"Returns the robot point to line residual");
    dqkinematics_py.def_static("point_to_plane_distance_jacobian", &DQ_Kinematics::point_to_plane_distance_jacobian,"Returns the robot point to plane distance Jacobian");
    dqkinematics_py.def_static("point_to_plane_residual",          &DQ_Kinematics::point_to_plane_residual,"Returns the robot point to plane residual");
    dqkinematics_py.def_static("line_to_point_distance_jacobian",  &DQ_Kinematics::line_to_point_distance_jacobian,"Returns the robot line to point distance Jacobian");
    dqkinematics_py.def_static("line_to_point_residual",           &DQ_Kinematics::line_to_point_residual,"Returns the robot line to point residual");
    dqkinematics_py.def_static("line_to_line_distance_jacobian",   &DQ_Kinematics::line_to_line_distance_jacobian,"Returns the robot line to line distance Jacobian");
    dqkinematics_py.def_static("line_to_line_residual",           &DQ_Kinematics::line_to_line_residual,        "Returns the robot line to line residual");
    dqkinematics_py.def_static("plane_to_point_distance_jacobian", &DQ_Kinematics::plane_to_point_distance_jacobian,"Returns the robot plane to point distance Jacobian");
    dqkinematics_py.def_static("plane_to_point_residual",          &DQ_Kinematics::plane_to_point_residual,"Returns the robot plane to point residual");
    dqkinematics_py.def_static("line_to_line_angle_jacobian",      &DQ_Kinematics::line_to_line_angle_jacobian,"Returns the line to line angle Jacobian");
    dqkinematics_py.def_static("line_to_line_angle_residual",      &DQ_Kinematics::line_to_line_angle_residual,"Returns the line to line angle residual");
    dqkinematics_py.def_static("line_segment_to_line_segment_distance_jacobian", &DQ_Kinematics::line_segment_to_line_segment_distance_jacobian, "Returns the line segment to line segment distance Jacobian");
}
