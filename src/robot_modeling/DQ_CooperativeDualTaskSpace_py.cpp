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

void init_DQ_CooperativeDualTaskSpace_py(py::module& m)
{
    /*****************************************************
     *  DQ CooperativeDualTaskSpace
     * **************************************************/
    py::class_<DQ_CooperativeDualTaskSpace> dqcooperativedualtaskspace(m, "DQ_CooperativeDualTaskSpace");
    dqcooperativedualtaskspace.def(py::init<DQ_Kinematics*, DQ_Kinematics*>());
    dqcooperativedualtaskspace.def("pose1",                  &DQ_CooperativeDualTaskSpace::pose1, "Returns the first robot's pose");
    dqcooperativedualtaskspace.def("pose2",                  &DQ_CooperativeDualTaskSpace::pose2,"Returns the second robot's pose");
    dqcooperativedualtaskspace.def("absolute_pose",          &DQ_CooperativeDualTaskSpace::absolute_pose,"Returns the absolute pose");
    dqcooperativedualtaskspace.def("relative_pose",          &DQ_CooperativeDualTaskSpace::relative_pose,"Returns the relative pose");
    dqcooperativedualtaskspace.def("pose_jacobian1",         &DQ_CooperativeDualTaskSpace::pose_jacobian1,"Returns the pose Jacobian of the first robot");
    dqcooperativedualtaskspace.def("pose_jacobian2",         &DQ_CooperativeDualTaskSpace::pose_jacobian2,"Returns the pose Jacobian of the second robot");
    dqcooperativedualtaskspace.def("absolute_pose_jacobian", &DQ_CooperativeDualTaskSpace::absolute_pose_jacobian,"Returns the absolute pose Jacobian");
    dqcooperativedualtaskspace.def("relative_pose_jacobian", &DQ_CooperativeDualTaskSpace::relative_pose_jacobian,"Returns the relative pose Jacobian");
}
