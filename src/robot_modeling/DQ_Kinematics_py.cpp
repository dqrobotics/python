#include "../dqrobotics_module.h"

void init_DQ_Kinematics_py(py::module& m)
{
    /*****************************************************
     *  DQ Kinematics
     * **************************************************/
    py::class_<DQ_Kinematics> dqkinematics_py(m, "DQ_Kinematics");
    //dqkinematics_py.def(py::init<>());

    dqkinematics_py.def("pose_jacobian",                           (MatrixXd (DQ_Kinematics::*)(const VectorXd&) const)&DQ_Kinematics::pose_jacobian, "Returns the pose Jacobian");
    dqkinematics_py.def("get_dim_configuration_space",             &DQ_Kinematics::get_dim_configuration_space, "Returns the dimension of the configuration space");

    dqkinematics_py.def_static("distance_jacobian",                &DQ_Kinematics::distance_jacobian,     "Returns the distance Jacobian");
    dqkinematics_py.def_static("translation_jacobian",             &DQ_Kinematics::translation_jacobian,  "Returns the translation Jacobian");
    dqkinematics_py.def_static("rotation_jacobian",                &DQ_Kinematics::rotation_jacobian,     "Returns the rotation Jacobian");
    dqkinematics_py.def_static("line_jacobian",                    &DQ_Kinematics::line_jacobian,         "Returns the line Jacobian");
    dqkinematics_py.def_static("plane_jacobian",                   &DQ_Kinematics::plane_jacobian,        "Returns the plane Jacobian");
    dqkinematics_py.def_static("point_to_point_distance_jacobian", &DQ_Kinematics::point_to_point_distance_jacobian,"Returns the robot point to point distance Jacobian");
    dqkinematics_py.def_static("point_to_point_residual",          &DQ_Kinematics::point_to_point_residual,"Returns the robot point to point residual");
    dqkinematics_py.def_static("point_to_line_distance_jacobian",  &DQ_Kinematics::point_to_line_distance_jacobian,"Returns the robot point to line distance Jacobian");
    dqkinematics_py.def_static("point_to_line_residual",           &DQ_Kinematics::point_to_line_residual,"Returns the robot point to line residual");
    dqkinematics_py.def_static("point_to_plane_distance_jacobian", &DQ_Kinematics::point_to_plane_distance_jacobian,"Returns the robot point to plane distance Jacobian");
    dqkinematics_py.def_static("point_to_plane_residual",          &DQ_Kinematics::point_to_plane_residual,"Returns the robot point to plane residual");
    dqkinematics_py.def_static("line_to_point_distance_jacobian",  &DQ_Kinematics::line_to_point_distance_jacobian,"Returns the robot line to point distance Jacobian");
    dqkinematics_py.def_static("line_to_point_residual",           &DQ_Kinematics::line_to_point_residual,"Returns the robot line to point residual");
    dqkinematics_py.def_static("line_to_line_distance_jacobian",   &DQ_Kinematics::line_to_line_distance_jacobian,"Returns the robot line to line distance Jacobian");
    dqkinematics_py.def_static("line_to_point_residual",           &DQ_Kinematics::line_to_point_residual,        "Returns the robot line to line residual");
    dqkinematics_py.def_static("plane_to_point_distance_jacobian", &DQ_Kinematics::plane_to_point_distance_jacobian,"Returns the robot plane to point distance Jacobian");
    dqkinematics_py.def_static("plane_to_point_residual",          &DQ_Kinematics::plane_to_point_residual,"Returns the robot plane to point residual");
}
