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
