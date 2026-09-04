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

/**
 * @brief Binds `DQ_Kinematics`, the common base class for robot models in DQ
 * Robotics that stores reference/base frames and exposes task-space Jacobian
 * utilities, to the Python module @p m.
 */
void init_DQ_Kinematics_py(py::module& m)
{
    py::class_<
            DQ_Kinematics,
            std::shared_ptr<DQ_Kinematics>
            > dqkinematics_py(
                m,
                "DQ_Kinematics",
                "Abstract class that defines an interface to implement robot kinematics. "
                "It stores the reference and base frames, the dimension of the configuration space, "
                "and static operations that derive task-space Jacobians from a pose Jacobian represented in dual quaternion form.");

    dqkinematics_py.def(
        "get_dim_configuration_space",
        &DQ_Kinematics::get_dim_configuration_space,
        "Returns the dimension of the configuration space as the number of generalized coordinates of the model.");
    dqkinematics_py.def(
        "get_reference_frame",
        &DQ_Kinematics::get_reference_frame,
        "Returns the current reference frame as a unit dual quaternion.");
    dqkinematics_py.def(
        "set_reference_frame",
        &DQ_Kinematics::set_reference_frame,
        py::arg("get_reference_frame"),
        "Sets the reference frame used by the forward kinematics and Jacobian methods.");
    dqkinematics_py.def(
        "get_base_frame",
        &DQ_Kinematics::get_base_frame,
        "Returns the current base frame as a unit dual quaternion.");
    dqkinematics_py.def(
        "set_base_frame",
        &DQ_Kinematics::set_base_frame,
        py::arg("get_base_frame"),
        "Sets the physical base frame of the robot in the workspace.");

    dqkinematics_py.def_static(
        "distance_jacobian",
        &DQ_Kinematics::distance_jacobian,
        py::arg("pose_jacobian"),
        py::arg("pose"),
        "Computes the Jacobian of the squared distance from the pose origin to the reference-frame origin from a pose Jacobian.");
    dqkinematics_py.def_static(
        "translation_jacobian",
        &DQ_Kinematics::translation_jacobian,
        py::arg("pose_jacobian"),
        py::arg("pose"),
        "Computes the translation Jacobian from a pose Jacobian so that vec4(translation_dot) = J * q_dot.");
    dqkinematics_py.def_static(
        "rotation_jacobian",
        &DQ_Kinematics::rotation_jacobian,
        py::arg("pose_jacobian"),
        "Extracts the rotation Jacobian from a pose Jacobian so that vec4(rotation_dot) = J * q_dot.");
    dqkinematics_py.def_static(
        "line_jacobian",
        &DQ_Kinematics::line_jacobian,
        py::arg("pose_jacobian"),
        py::arg("pose"),
        py::arg("line_direction"),
        "Computes the Jacobian of a line obtained by rigidly attaching the local line direction to the given pose.");
    dqkinematics_py.def_static(
        "plane_jacobian",
        &DQ_Kinematics::plane_jacobian,
        py::arg("pose_jacobian"),
        py::arg("pose"),
        py::arg("plane_normal"),
        "Computes the Jacobian of a plane obtained by rigidly attaching the local plane normal to the given pose.");
    dqkinematics_py.def_static(
        "distance_jacobian_derivative",
        &DQ_Kinematics::distance_jacobian_derivative,
        py::arg("pose_jacobian"),
        py::arg("pose_jacobian_derivative"),
        py::arg("pose"),
        py::arg("q_dot"),
        "Computes the time derivative of the squared-distance Jacobian.");
    dqkinematics_py.def_static(
        "translation_jacobian_derivative",
        &DQ_Kinematics::translation_jacobian_derivative,
        py::arg("pose_jacobian"),
        py::arg("pose_jacobian_derivative"),
        py::arg("pose"),
        py::arg("q_dot"),
        "Computes the time derivative of the translation Jacobian.");
    dqkinematics_py.def_static(
        "rotation_jacobian_derivative",
        &DQ_Kinematics::rotation_jacobian_derivative,
        py::arg("pose_jacobian_derivative"),
        "Extracts the rotation-Jacobian derivative from a pose-Jacobian derivative.");
    dqkinematics_py.def_static(
        "line_jacobian_derivative",
        &DQ_Kinematics::line_jacobian_derivative,
        py::arg("pose_jacobian"),
        py::arg("pose_jacobian_derivative"),
        py::arg("pose"),
        py::arg("line_direction"),
        py::arg("q_dot"),
        "Computes the time derivative of a line Jacobian.");
    dqkinematics_py.def_static(
        "plane_jacobian_derivative",
        &DQ_Kinematics::plane_jacobian_derivative,
        py::arg("pose_jacobian"),
        py::arg("pose_jacobian_derivative"),
        py::arg("pose"),
        py::arg("plane_normal"),
        py::arg("q_dot"),
        "Computes the time derivative of a plane Jacobian.");
    dqkinematics_py.def_static(
        "point_to_point_distance_jacobian",
        &DQ_Kinematics::point_to_point_distance_jacobian,
        py::arg("translation_jacobian"),
        py::arg("robot_point"),
        py::arg("workspace_point"),
        "Computes the squared point-to-point distance Jacobian.");
    dqkinematics_py.def_static(
        "point_to_point_residual",
        &DQ_Kinematics::point_to_point_residual,
        py::arg("robot_point"),
        py::arg("workspace_point"),
        py::arg("workspace_point_derivative"),
        "Computes the residual term of the squared point-to-point distance dynamics.");
    dqkinematics_py.def_static(
        "point_to_line_distance_jacobian",
        &DQ_Kinematics::point_to_line_distance_jacobian,
        py::arg("translation_jacobian"),
        py::arg("robot_point"),
        py::arg("workspace_line"),
        "Computes the squared point-to-line distance Jacobian.");
    dqkinematics_py.def_static(
        "point_to_line_residual",
        &DQ_Kinematics::point_to_line_residual,
        py::arg("robot_point"),
        py::arg("workspace_line"),
        py::arg("workspace_line_derivative"),
        "Computes the residual term of the squared point-to-line distance dynamics.");
    dqkinematics_py.def_static(
        "point_to_plane_distance_jacobian",
        &DQ_Kinematics::point_to_plane_distance_jacobian,
        py::arg("translation_jacobian"),
        py::arg("robot_point"),
        py::arg("workspace_plane"),
        "Computes the squared point-to-plane distance Jacobian.");
    dqkinematics_py.def_static(
        "point_to_plane_residual",
        &DQ_Kinematics::point_to_plane_residual,
        py::arg("translation"),
        py::arg("plane_derivative"),
        "Computes the residual term of the squared point-to-plane distance dynamics.");
    dqkinematics_py.def_static(
        "line_to_point_distance_jacobian",
        &DQ_Kinematics::line_to_point_distance_jacobian,
        py::arg("line_jacobian"),
        py::arg("robot_line"),
        py::arg("workspace_point"),
        "Computes the squared line-to-point distance Jacobian.");
    dqkinematics_py.def_static(
        "line_to_point_residual",
        &DQ_Kinematics::line_to_point_residual,
        py::arg("robot_line"),
        py::arg("workspace_point"),
        py::arg("workspace_point_derivative"),
        "Computes the residual term of the squared line-to-point distance dynamics.");
    dqkinematics_py.def_static(
        "line_to_line_distance_jacobian",
        &DQ_Kinematics::line_to_line_distance_jacobian,
        py::arg("line_jacobian"),
        py::arg("robot_line"),
        py::arg("workspace_line"),
        "Computes the squared line-to-line distance Jacobian.");
    dqkinematics_py.def_static(
        "line_to_line_residual",
        &DQ_Kinematics::line_to_line_residual,
        py::arg("robot_line"),
        py::arg("workspace_line"),
        py::arg("workspace_line_derivative"),
        "Computes the residual term of the squared line-to-line distance dynamics.");
    dqkinematics_py.def_static(
        "plane_to_point_distance_jacobian",
        &DQ_Kinematics::plane_to_point_distance_jacobian,
        py::arg("plane_jacobian"),
        py::arg("workspace_point"),
        "Computes the squared plane-to-point distance Jacobian.");
    dqkinematics_py.def_static(
        "plane_to_point_residual",
        &DQ_Kinematics::plane_to_point_residual,
        py::arg("robot_plane"),
        py::arg("workspace_point_derivative"),
        "Computes the residual term of the squared plane-to-point distance dynamics.");
    dqkinematics_py.def_static(
        "line_to_line_angle_jacobian",
        &DQ_Kinematics::line_to_line_angle_jacobian,
        py::arg("line_jacobian"),
        py::arg("robot_line"),
        py::arg("workspace_line"),
        "Computes the Jacobian of the line-to-line angle objective.");
    dqkinematics_py.def_static(
        "line_to_line_angle_residual",
        &DQ_Kinematics::line_to_line_angle_residual,
        py::arg("robot_line"),
        py::arg("workspace_line"),
        py::arg("workspace_line_derivative"),
        "Computes the residual term of the line-to-line angle objective.");
    dqkinematics_py.def_static(
        "line_segment_to_line_segment_distance_jacobian",
        &DQ_Kinematics::line_segment_to_line_segment_distance_jacobian,
        py::arg("line_jacobian"),
        py::arg("robot_point_1_translation_jacobian"),
        py::arg("robot_point_2_translation_jacobian"),
        py::arg("robot_line"),
        py::arg("robot_point_1"),
        py::arg("robot_point_2"),
        py::arg("workspace_line"),
        py::arg("workspace_point_1"),
        py::arg("workspace_point_2"),
        "Computes a squared-distance Jacobian between two line segments by selecting the appropriate active-constraint formulation.");
}
