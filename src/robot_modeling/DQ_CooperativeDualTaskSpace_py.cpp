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
 * @brief Binds `DQ_CooperativeDualTaskSpace`, which implements the cooperative
 * dual task-space formulation for two robots, to the Python module @p m.
 */
void init_DQ_CooperativeDualTaskSpace_py(py::module& m)
{
    py::class_<
            DQ_CooperativeDualTaskSpace
            > dqcooperativedualtaskspace(
                m,
                "DQ_CooperativeDualTaskSpace",
                "Implements the cooperative dual task-space formulation for two robots. The cooperative variables are the absolute pose and the relative pose of the two end effectors, together with their corresponding Jacobians.");
    dqcooperativedualtaskspace.def(
        py::init<DQ_Kinematics*, DQ_Kinematics*>(),
        py::arg("robot1"),
        py::arg("robot2"),
        "Constructs a cooperative dual task-space system from two robot models. The object does not take ownership of the provided pointers.");
    dqcooperativedualtaskspace.def(
        "pose1",
        &DQ_CooperativeDualTaskSpace::pose1,
        py::arg("theta"),
        "Returns the pose of the first end effector for the combined configuration vector theta = [q1; q2].");
    dqcooperativedualtaskspace.def(
        "pose2",
        &DQ_CooperativeDualTaskSpace::pose2,
        py::arg("theta"),
        "Returns the pose of the second end effector for the combined configuration vector theta = [q1; q2].");
    dqcooperativedualtaskspace.def(
        "absolute_pose",
        &DQ_CooperativeDualTaskSpace::absolute_pose,
        py::arg("theta"),
        "Computes the absolute pose of the cooperative system as the frame located midway between the two end effectors.");
    dqcooperativedualtaskspace.def(
        "relative_pose",
        &DQ_CooperativeDualTaskSpace::relative_pose,
        py::arg("theta"),
        "Computes the relative pose between the two end effectors. The returned dual quaternion maps the second end-effector frame to the first one.");
    dqcooperativedualtaskspace.def(
        "pose_jacobian1",
        &DQ_CooperativeDualTaskSpace::pose_jacobian1,
        py::arg("theta"),
        "Returns the pose Jacobian of the first robot end effector for the combined configuration vector theta = [q1; q2].");
    dqcooperativedualtaskspace.def(
        "pose_jacobian2",
        &DQ_CooperativeDualTaskSpace::pose_jacobian2,
        py::arg("theta"),
        "Returns the pose Jacobian of the second robot end effector for the combined configuration vector theta = [q1; q2].");
    dqcooperativedualtaskspace.def(
        "absolute_pose_jacobian",
        &DQ_CooperativeDualTaskSpace::absolute_pose_jacobian,
        py::arg("theta"),
        "Computes the Jacobian of the absolute pose.");
    dqcooperativedualtaskspace.def(
        "relative_pose_jacobian",
        &DQ_CooperativeDualTaskSpace::relative_pose_jacobian,
        py::arg("theta"),
        "Computes the Jacobian of the relative pose.");
}
