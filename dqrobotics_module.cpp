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

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

#include <dqrobotics/DQ.h>

#include <dqrobotics/utils/DQ_LinearAlgebra.h>
#include <dqrobotics/utils/DQ_Geometry.h>

#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <dqrobotics/robot_modeling/DQ_MobileBase.h>
#include <dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include <dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h>
#include <dqrobotics/robot_modeling/DQ_WholeBody.h>

#include <dqrobotics/robots/Ax18ManipulatorRobot.h>
#include <dqrobotics/robots/BarrettWamArmRobot.h>
#include <dqrobotics/robots/ComauSmartSixRobot.h>
#include <dqrobotics/robots/KukaLw4Robot.h>

#include <dqrobotics/interfaces/vrep_interface.h>

using namespace DQ_robotics;
using namespace Eigen;

PYBIND11_MODULE(dqrobotics, m) {
    /*****************************************************
     *  DQ
     * **************************************************/
    py::class_<DQ> dq(m, "DQ");
    dq.def(py::init<double, double, double, double, double, double, double, double>());
    dq.def(py::init<VectorXd>());
    ///Members
    dq.def_readwrite("q", &DQ::q);
    ///Static Members
    dq.def_readonly_static("i",&DQ::i);
    dq.def_readonly_static("j",&DQ::j);
    dq.def_readonly_static("k",&DQ::k);
    dq.def_readonly_static("E",&DQ::E);
    ///Methods
    dq.def("P"                   ,&DQ::P,                     "Retrieves the primary part of a DQ.");
    dq.def("D"                   ,&DQ::D,                     "Retrieves the dual part of a DQ.");
    dq.def("Re"                  ,&DQ::Re,                    "Retrieves the real part of a DQ.");
    dq.def("Im"                  ,&DQ::Im,                    "Retrieves the imaginary part of a DQ.");
    dq.def("conj"                ,&DQ::conj,                  "Retrieves the conjugate of a DQ.");
    dq.def("norm"                ,&DQ::norm,                  "Retrieves the norm of a DQ.");
    dq.def("inv"                 ,&DQ::inv,                   "Retrieves the inverse of a DQ.");
    dq.def("translation"         ,&DQ::translation,           "Retrieves the translation represented by a unit DQ.");
    dq.def("rotation"            ,&DQ::rotation,              "Retrieves the rotation represented by a unit DQ.");
    dq.def("rotation_axis"       ,&DQ::rotation_axis,         "Retrieves the rotation axis represented by a unit DQ.");
    dq.def("rotation_angle"      ,&DQ::rotation_angle,        "Retrieves the rotation angle represented by a unit DQ.");
    dq.def("log"                 ,&DQ::log,                   "Retrieves the logarithm of a DQ.");
    dq.def("exp"                 ,&DQ::exp,                   "Retrieves the exp of a DQ.");
    dq.def("pow"                 ,&DQ::pow,                   "Retrieves the pow of a DQ.");
    dq.def("tplus"               ,&DQ::tplus,                 "Retrieves the tplus operators for a DQ.");
    dq.def("pinv"                ,&DQ::pinv ,                 "Retrieves the pinv of a DQ.");
    //NOT DEFINED IN CPP dq.def("dec_mult"           ,&DQ::dec_mult,"Retrieves the dec mult of a DQ.");
    dq.def("hamiplus4"           ,&DQ::hamiplus4,             "Retrieves the H+ operator for the primary part of a DQ.");
    dq.def("haminus4"            ,&DQ::haminus4,              "Retrieves the H- operator for the primary part of a DQ.");
    dq.def("hamiplus8"           ,&DQ::hamiplus8,             "Retrieves the H+ operator for a DQ.");
    dq.def("haminus8"            ,&DQ::haminus8,              "Retrieves the H- operator for a DQ.");
    dq.def("vec4"                ,&DQ::vec4,                  "Retrieves the primary part of a DQ as a vector.");
    dq.def("vec8"                ,&DQ::vec8,                  "Retrieves the DQ as a vector.");
    dq.def("normalize"           ,&DQ::normalize,             "Returns a normalized DQ.");
    dq.def("__repr__"            ,&DQ::to_string);
    dq.def("generalized_jacobian",&DQ::generalized_jacobian,  "Retrieves the generalized Jacobian of a DQ.");
    dq.def("sharp"               ,&DQ::sharp,                 "Retrieves the sharp of a DQ");
    dq.def("Ad"                  ,&DQ::Ad,                    "Retrieves the adjoint transformation of a DQ.");
    dq.def("Adsharp"             ,&DQ::Adsharp,               "Retrieves the adjoint sharp transformation of a DQ.");
    //dq.def("cross"               ,&DQ::cross,                 "Returns the result of the cross operation with another DQ");
    ////Deprecated
    //dq.def("generalizedJacobian", &DQ::generalizedJacobian,"Retrieves the generalized Jacobian of a DQ.");

    ///Operators
    //Self
    dq.def(py::self + py::self);
    dq.def(py::self * py::self);
    dq.def(py::self - py::self);
    dq.def(py::self == py::self);
    dq.def(py::self != py::self);
    //Float
    dq.def(float()  * py::self);
    dq.def(py::self * float());
    dq.def(float()  + py::self);
    dq.def(py::self + float());
    dq.def(float()  - py::self);
    dq.def(py::self - float());
    dq.def(float()  == py::self);
    dq.def(py::self == float());
    dq.def(float()  != py::self);
    dq.def(py::self != float());

    ///Namespace Functions
    m.def("C8"                  ,&DQ_robotics::C8,                   "Returns the C8 matrix.");
    m.def("C4"                  ,&DQ_robotics::C4,                   "Returns the C4 matrix.");
    m.def("P"                   ,&DQ_robotics::P,                    "Retrieves the primary part of a DQ.");
    m.def("D"                   ,&DQ_robotics::D,                    "Retrieves the dual part of a DQ.");
    m.def("Re"                  ,&DQ_robotics::Re,                   "Retrieves the real part of a DQ.");
    m.def("Im"                  ,&DQ_robotics::Im,                   "Retrieves the imaginary part of a DQ.");
    m.def("conj"                ,&DQ_robotics::conj,                 "Retrieves the conjugate of a DQ.");
    m.def("norm"                ,&DQ_robotics::norm,                 "Retrieves the norm of a DQ.");
    m.def("inv"                 ,&DQ_robotics::inv,                  "Retrieves the inverse of a DQ.");
    m.def("translation"         ,&DQ_robotics::translation,          "Retrieves the translation represented by a unit DQ.");
    m.def("rotation"            ,&DQ_robotics::rotation,             "Retrieves the rotation represented by a unit DQ.");
    m.def("rotation_axis"       ,&DQ_robotics::rotation_axis,        "Retrieves the rotation axis represented by a unit DQ.");
    m.def("rotation_angle"      ,&DQ_robotics::rotation_angle,       "Retrieves the rotation angle represented by a unit DQ.");
    m.def("log"                 ,&DQ_robotics::log,                  "Retrieves the logarithm of a DQ.");
    m.def("exp"                 ,&DQ_robotics::exp,                  "Retrieves the exp of a DQ.");
    m.def("pow"                 ,&DQ_robotics::pow,                  "Retrieves the pow of a DQ.");
    m.def("tplus"               ,&DQ_robotics::tplus,                "Retrieves the tplus operators for a DQ.");
    m.def("pinv"                ,(DQ (*) (const DQ&)) &DQ_robotics::pinv ,"Retrieves the pinv of a DQ.");
    m.def("dec_mult"            ,&DQ_robotics::dec_mult,             "Retrieves the dec mult of a DQ.");
    m.def("hamiplus4"           ,&DQ_robotics::hamiplus4,            "Retrieves the H+ operator for the primary part of a DQ.");
    m.def("haminus4"            ,&DQ_robotics::haminus4,             "Retrieves the H- operator for the primary part of a DQ.");
    m.def("hamiplus8"           ,&DQ_robotics::hamiplus8,            "Retrieves the H+ operator for a DQ.");
    m.def("haminus8"            ,&DQ_robotics::haminus8,             "Retrieves the H- operator for a DQ.");
    m.def("vec4"                ,&DQ_robotics::vec4,                 "Retrieves the primary part of a DQ as a vector.");
    m.def("vec8"                ,&DQ_robotics::vec8,                 "Retrieves the DQ as a vector.");
    m.def("normalize"           ,&DQ_robotics::normalize,            "Returns a normalized DQ.");
    m.def("generalized_jacobian",&DQ_robotics::generalized_jacobian, "Retrieves the generalized Jacobian of a DQ.");
    m.def("sharp"               ,&DQ_robotics::sharp,                "Returns the sharp DQ");
    m.def("crossmatrix4"        ,&DQ_robotics::crossmatrix4,         "Returns the crossmatrix4 operator.");
    m.def("Ad"                  ,&DQ_robotics::Ad,                   "Retrieves the adjoint transformation of a DQ.");
    m.def("Adsharp"             ,&DQ_robotics::Adsharp,              "Retrieves the adjoint sharp transformation of a DQ.");
    m.def("cross"               ,&DQ_robotics::cross,                "Returns the result of the cross product between two DQ");
    m.def("dot"                 ,&DQ_robotics::dot,                  "Returns the result of the dot product between two DQ");
    ////DEPRECATED
    //m.def("generalizedJacobian",&DQ_robotics::generalizedJacobian,"Retrieves the generalized Jacobian of a DQ.");

    ///Namespace readonly
    m.attr("DQ_threshold") = DQ_threshold;
    m.attr("i_")           = DQ_robotics::i_;
    m.attr("j_")           = DQ_robotics::j_;
    m.attr("k_")           = DQ_robotics::k_;

    ////DEPRECATED

    /*****************************************************
     *  Utils
     * **************************************************/
    //dqrobotics/utils/
    py::module utils_py = m.def_submodule("utils","A submodule of dqrobotics");

    /*****************************************************
     *  DQ_LinearAlgebra
     * **************************************************/
    //#include<dqrobotics/utils/DQ_LinearAlgebra.h>
    py::module linearalgebra_py = utils_py.def_submodule("DQ_LinearAlgebra","A submodule of utils");
    linearalgebra_py.def("pinv", (MatrixXd (*) (const MatrixXd&))&DQ_robotics::pinv, "Retrieves the pseudo-inverse of the input matrix");

    /*****************************************************
     *  DQ_Geometry
     * **************************************************/
    //#include<dqrobotics/utils/DQ_Geometry.h>
    py::class_<DQ_Geometry> geometry_py = m.def_submodule("DQ_Geometry","A submodule of utils");
    geometry_py.def_static("point_to_point_squared_distance",  &DQ_Geometry::point_to_point_squared_distance, "Returns the squared distance between two points");
    geometry_py.def_static("point_to_line_squared_distance",   &DQ_Geometry::point_to_line_squared_distance,  "Returns the squared distance between a point and a line");
    geometry_py.def_static("point_to_plane_distance",          &DQ_Geometry::point_to_plane_distance,         "Returns the distance between a point and a plane");
    geometry_py.def_static("line_to_line_squared_distance",    &DQ_Geometry::line_to_line_squared_distance,   "Returns the squared distance between two lines");

    /*****************************************************
     *  Robots Kinematic Models
     * **************************************************/
    py::module robots_py = m.def_submodule("robots", "A submodule of dqrobotics");

    //#include <dqrobotics/robots/Ax18ManipulatorRobot.h>
    py::class_<Ax18ManipulatorRobot> ax18manipulatorrobot_py(robots_py, "Ax18ManipulatorRobot");
    ax18manipulatorrobot_py.def_static("kinematics",&Ax18ManipulatorRobot::kinematics,"Returns the kinematics of the Ax18ManipulatorRobot");

    //#include <dqrobotics/robots/BarrettWamArmRobot.h>
    py::class_<BarrettWamArmRobot> barrettwamarmrobot_py(robots_py, "BarrettWamArmRobot");
    barrettwamarmrobot_py.def_static("kinematics",&BarrettWamArmRobot::kinematics,"Returns the kinematics of the BarrettWamArmRobot");

    //#include <dqrobotics/robots/ComauSmartSixRobot.h>
    py::class_<ComauSmartSixRobot> comausmartsixrobot_py(robots_py, "ComauSmartSixRobot");
    comausmartsixrobot_py.def_static("kinematics",&ComauSmartSixRobot::kinematics,"Returns the kinematics of the ComauSmartSixRobot");

    //#include <dqrobotics/robots/KukaLw4Robot.h>
    py::class_<KukaLw4Robot> kukalw4robot_py(robots_py, "KukaLw4Robot");
    kukalw4robot_py.def_static("kinematics",&KukaLw4Robot::kinematics,"Returns the kinematics of the KukaLw4Robot");

    /*****************************************************
     *  Robot Modelling <dqrobotics/robot_modelling/...>
     * **************************************************/
    py::module robot_modeling = m.def_submodule("robot_modeling", "The robot_modeling submodule of dqrobotics");

    /*****************************************************
     *  DQ Kinematics
     * **************************************************/
    py::class_<DQ_Kinematics> dqkinematics_py(robot_modeling, "DQ_Kinematics");
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

    /*****************************************************
     *  DQ SerialManipulator
     * **************************************************/
    py::class_<DQ_SerialManipulator,DQ_Kinematics> dqserialmanipulator_py(robot_modeling, "DQ_SerialManipulator");
    dqserialmanipulator_py.def(py::init<MatrixXd, std::string>());
    dqserialmanipulator_py.def(py::init());
    ///Methods
    dqserialmanipulator_py.def("getDHMatrix",                 &DQ_SerialManipulator::getDHMatrix,"Gets the DH matrix.");
    dqserialmanipulator_py.def("theta",                       &DQ_SerialManipulator::theta,"Retrieves the vector of thetas.");
    dqserialmanipulator_py.def("d",                           &DQ_SerialManipulator::d,"Retrieves the vector d.");
    dqserialmanipulator_py.def("a",                           &DQ_SerialManipulator::a,"Retrieves the vector a.");
    dqserialmanipulator_py.def("alpha",                       &DQ_SerialManipulator::alpha,"Retrieves the vector of alphas.");
    //dqserialmanipulator_py.def("dummy",                       &DQ_SerialManipulator::dummy,"Retrieves the vector of dummies.");
    //dqserialmanipulator_py.def("set_dummy",                   &DQ_SerialManipulator::set_dummy,"Sets the vector of dummies.");
    //dqserialmanipulator_py.def("n_dummy",                     &DQ_SerialManipulator::n_dummy,"Retrieves the number of dummy joints.");
    dqserialmanipulator_py.def("get_dim_configuration_space", &DQ_SerialManipulator::get_dim_configuration_space,"Retrieves the number of links.");
    dqserialmanipulator_py.def("convention",                  &DQ_SerialManipulator::convention,"Retrieves the DH convention.");
    dqserialmanipulator_py.def("base_frame",                  &DQ_SerialManipulator::base_frame,"Retrieves the base.");
    dqserialmanipulator_py.def("reference_frame",             &DQ_SerialManipulator::reference_frame,"Retrieves the reference frame");
    dqserialmanipulator_py.def("effector",                    &DQ_SerialManipulator::effector,"Retrieves the effector.");
    dqserialmanipulator_py.def("set_base_frame",              &DQ_SerialManipulator::set_base_frame,"Sets the base.");
    dqserialmanipulator_py.def("set_reference_frame",         &DQ_SerialManipulator::set_reference_frame,"Sets the reference frame");
    dqserialmanipulator_py.def("set_effector",                &DQ_SerialManipulator::set_effector,"Sets the effector.");
    dqserialmanipulator_py.def("raw_fkm",                     (DQ (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::raw_fkm,"Gets the raw fkm.");
    dqserialmanipulator_py.def("raw_fkm",                     (DQ (DQ_SerialManipulator::*)(const VectorXd&,const int&) const)&DQ_SerialManipulator::raw_fkm,"Gets the raw fkm.");
    dqserialmanipulator_py.def("fkm",                         (DQ (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::fkm,"Gets the fkm.");
    dqserialmanipulator_py.def("fkm",                         (DQ (DQ_SerialManipulator::*)(const VectorXd&,const int&) const)&DQ_SerialManipulator::fkm,"Gets the fkm.");
    dqserialmanipulator_py.def("dh2dq",                       &DQ_SerialManipulator::dh2dq,"Returns a link's DH transformation as a DQ.");
    dqserialmanipulator_py.def("get_z",                       &DQ_SerialManipulator::get_z,"Returns the z of a transformation.");
    dqserialmanipulator_py.def("raw_pose_jacobian",           &DQ_SerialManipulator::raw_pose_jacobian, "Returns the pose Jacobian up to a given link without base and end-effector displacements.");
    dqserialmanipulator_py.def("pose_jacobian",               (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&) const)&DQ_SerialManipulator::pose_jacobian,"Returns the pose Jacobian");
    dqserialmanipulator_py.def("pose_jacobian",               (MatrixXd (DQ_SerialManipulator::*)(const VectorXd&, const int&) const)&DQ_SerialManipulator::pose_jacobian,"Returns the pose Jacobian up to a given link");
    dqserialmanipulator_py.def("pose_jacobian_derivative",    &DQ_SerialManipulator::pose_jacobian_derivative,"Returns the derivative of the pose Jacobian");

    /*****************************************************
     *  DQ CooperativeDualTaskSpace
     * **************************************************/
    py::class_<DQ_CooperativeDualTaskSpace> dqcooperativedualtaskspace(robot_modeling, "DQ_CooperativeDualTaskSpace");
    dqcooperativedualtaskspace.def(py::init<DQ_Kinematics*, DQ_Kinematics*>());
    dqcooperativedualtaskspace.def("pose1",                  &DQ_CooperativeDualTaskSpace::pose1, "Returns the first robot's pose");
    dqcooperativedualtaskspace.def("pose2",                  &DQ_CooperativeDualTaskSpace::pose2,"Returns the second robot's pose");
    dqcooperativedualtaskspace.def("absolute_pose",          &DQ_CooperativeDualTaskSpace::absolute_pose,"Returns the absolute pose");
    dqcooperativedualtaskspace.def("relative_pose",          &DQ_CooperativeDualTaskSpace::relative_pose,"Returns the relative pose");
    dqcooperativedualtaskspace.def("pose_jacobian1",         &DQ_CooperativeDualTaskSpace::pose_jacobian1,"Returns the pose Jacobian of the first robot");
    dqcooperativedualtaskspace.def("pose_jacobian2",         &DQ_CooperativeDualTaskSpace::pose_jacobian2,"Returns the pose Jacobian of the second robot");
    dqcooperativedualtaskspace.def("absolute_pose_jacobian", &DQ_CooperativeDualTaskSpace::absolute_pose_jacobian,"Returns the absolute pose Jacobian");
    dqcooperativedualtaskspace.def("relative_pose_jacobian", &DQ_CooperativeDualTaskSpace::relative_pose_jacobian,"Returns the relative pose Jacobian");

    /*****************************************************
     *  DQ MobileBase
     * **************************************************/
    py::class_<DQ_MobileBase,DQ_Kinematics> dqmobilebase_py(robot_modeling,"DQ_MobileBase");
    dqmobilebase_py.def("set_frame_displacement", &DQ_MobileBase::set_frame_displacement,"Set the frame displacement");
    dqmobilebase_py.def("frame_displacement",     &DQ_MobileBase::frame_displacement,    "Get the frame displacement");

    /*****************************************************
     *  DQ HolonomicBase
     * **************************************************/
    py::class_<DQ_HolonomicBase,DQ_MobileBase> dqholonomicbase_py(robot_modeling,"DQ_HolonomicBase");
    dqholonomicbase_py.def(py::init());
    dqholonomicbase_py.def("fkm",                        &DQ_HolonomicBase::fkm,"Returns the base's fkm");
    dqholonomicbase_py.def("pose_jacobian",              &DQ_HolonomicBase::pose_jacobian,"Returns the base's Jacobian");
    dqholonomicbase_py.def("get_dim_configuration_space",&DQ_HolonomicBase::get_dim_configuration_space,"Returns the size of the configuration space");
    dqholonomicbase_py.def("raw_fkm",                    &DQ_HolonomicBase::raw_fkm,"Returns the raw fkm");
    dqholonomicbase_py.def("raw_pose_jacobian",          &DQ_HolonomicBase::raw_pose_jacobian,"Return the raw ose Jacobian");

    /*****************************************************
     *  DQ DifferentialDriveRobot
     * **************************************************/
    py::class_<DQ_DifferentialDriveRobot,DQ_HolonomicBase> dqdifferentialdriverobot_py(robot_modeling,"DQ_DifferentialDriveRobot");
    dqdifferentialdriverobot_py.def(py::init<const double&, const double&>());
    dqdifferentialdriverobot_py.def("constraint_jacobian", &DQ_DifferentialDriveRobot::constraint_jacobian, "Returns the constraint Jacobian");
    dqdifferentialdriverobot_py.def("pose_jacobian",       &DQ_DifferentialDriveRobot::pose_jacobian,       "Returns the pose Jacobian");

    /*****************************************************
     *  DQ WholeBody
     * **************************************************/
    py::class_<DQ_WholeBody,DQ_Kinematics> dqwholebody_py(robot_modeling,"DQ_WholeBody");
    dqwholebody_py.def(py::init<DQ_Kinematics*>());
    dqwholebody_py.def("add",&DQ_WholeBody::add,"Adds a DQ_Kinematics pointer to the kinematic chain.");
    dqwholebody_py.def("fkm",(DQ (DQ_WholeBody::*)(const VectorXd&) const)&DQ_WholeBody::fkm,"Gets the fkm.");
    dqwholebody_py.def("fkm",(DQ (DQ_WholeBody::*)(const VectorXd&,const int&) const)&DQ_WholeBody::fkm,"Gets the fkm.");
    dqwholebody_py.def("get_dim_configuration_space",&DQ_WholeBody::get_dim_configuration_space,"Gets the dimention of the configuration space");
    dqwholebody_py.def("pose_jacobian",&DQ_WholeBody::pose_jacobian,"Returns the combined pose Jacobian");


    /*****************************************************
     *  Interfaces Submodule
     * **************************************************/
    py::module interfaces_py = m.def_submodule("interfaces", "A submodule of dqrobotics");

    /*****************************************************
     *  VrepInterface
     * **************************************************/
    py::class_<VrepInterface> vrepinterface_py(interfaces_py,"VrepInterface");
    vrepinterface_py.def(py::init<>());
    vrepinterface_py.def(py::init<std::atomic_bool*>());

    py::enum_<VrepInterface::OP_MODES>(vrepinterface_py, "OP_MODES")
            .value("OP_BUFFER",    VrepInterface::OP_MODES::OP_BUFFER)
            .value("OP_ONESHOT",   VrepInterface::OP_MODES::OP_ONESHOT)
            .value("OP_BLOCKING",  VrepInterface::OP_MODES::OP_BLOCKING)
            .value("OP_STREAMING", VrepInterface::OP_MODES::OP_STREAMING)
            .export_values();

    vrepinterface_py.def("connect",(bool (VrepInterface::*) (const int&, const int&, const int&))&VrepInterface::connect,"Connects to V-REP in local machine.");
    vrepinterface_py.def("connect",(bool (VrepInterface::*) (const std::string&, const int&, const int&, const int&))&VrepInterface::connect,"Connects to V-REP with a given ip.");

    vrepinterface_py.def("disconnect",    &VrepInterface::disconnect,"Disconnects from V-REP.");
    vrepinterface_py.def("disconnect_all",&VrepInterface::disconnect_all,"Disconnect all from V-REP");

    vrepinterface_py.def("start_simulation",&VrepInterface::start_simulation,"Start simulation");
    vrepinterface_py.def("stop_simulation", &VrepInterface::stop_simulation,"Stops simulation");

    vrepinterface_py.def("is_simulation_running",&VrepInterface::is_simulation_running,"Checks whether the simulation is running or not");

    vrepinterface_py.def("get_object_handle", &VrepInterface::get_object_handle,"Gets an object handle");
    vrepinterface_py.def("get_object_handles",&VrepInterface::get_object_handles,"Get object handles");

    vrepinterface_py.def("get_object_translation",(DQ (VrepInterface::*) (const int&, const int&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_translation,"Gets object translation.");
    vrepinterface_py.def("get_object_translation",(DQ (VrepInterface::*) (const std::string&, const int&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_translation,"Gets object translation.");
    vrepinterface_py.def("get_object_translation",(DQ (VrepInterface::*) (const int&, const std::string&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_translation,"Gets object translation.");
    vrepinterface_py.def("get_object_translation",(DQ (VrepInterface::*) (const std::string&, const std::string&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_translation,"Gets object translation.");

    vrepinterface_py.def("set_object_translation",(void (VrepInterface::*) (const int&, const int&, const DQ&, const VrepInterface::OP_MODES&) const)&VrepInterface::set_object_translation,"Sets object translation.");
    vrepinterface_py.def("set_object_translation",(void (VrepInterface::*) (const std::string&, const int&, const DQ&,  const VrepInterface::OP_MODES&))&VrepInterface::set_object_translation,"Sets object translation.");
    vrepinterface_py.def("set_object_translation",(void (VrepInterface::*) (const int&, const std::string&, const DQ&,  const VrepInterface::OP_MODES&))&VrepInterface::set_object_translation,"Sets object translation.");
    vrepinterface_py.def("set_object_translation",(void (VrepInterface::*) (const std::string&, const std::string&, const DQ&,  const VrepInterface::OP_MODES&))&VrepInterface::set_object_translation,"Sets object translation.");

    vrepinterface_py.def("get_object_rotation",(DQ (VrepInterface::*) (const int&, const int&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_rotation,"Gets object rotation.");
    vrepinterface_py.def("get_object_rotation",(DQ (VrepInterface::*) (const std::string&, const int&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_rotation,"Gets object rotation.");
    vrepinterface_py.def("get_object_rotation",(DQ (VrepInterface::*) (const int&, const std::string&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_rotation,"Gets object rotation.");
    vrepinterface_py.def("get_object_rotation",(DQ (VrepInterface::*) (const std::string&, const std::string&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_rotation,"Gets object rotation.");

    vrepinterface_py.def("set_object_rotation",(void (VrepInterface::*) (const int&, const int&, const DQ&, const VrepInterface::OP_MODES&) const)&VrepInterface::set_object_rotation,"Sets object rotation.");
    vrepinterface_py.def("set_object_rotation",(void (VrepInterface::*) (const std::string&, const int&, const DQ&,  const VrepInterface::OP_MODES&))&VrepInterface::set_object_rotation,"Sets object rotation.");
    vrepinterface_py.def("set_object_rotation",(void (VrepInterface::*) (const int&, const std::string&, const DQ&,  const VrepInterface::OP_MODES&))&VrepInterface::set_object_rotation,"Sets object rotation.");
    vrepinterface_py.def("set_object_rotation",(void (VrepInterface::*) (const std::string&, const std::string&, const DQ&,  const VrepInterface::OP_MODES&))&VrepInterface::set_object_rotation,"Sets object rotation.");

    vrepinterface_py.def("get_object_pose",(DQ (VrepInterface::*) (const int&, const int&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_pose,"Gets object pose.");
    vrepinterface_py.def("get_object_pose",(DQ (VrepInterface::*) (const std::string&, const int&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_pose,"Gets object pose.");
    vrepinterface_py.def("get_object_pose",(DQ (VrepInterface::*) (const int&, const std::string&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_pose,"Gets object pose.");
    vrepinterface_py.def("get_object_pose",(DQ (VrepInterface::*) (const std::string&, const std::string&, const VrepInterface::OP_MODES&))&VrepInterface::get_object_pose,"Gets object pose.");

    vrepinterface_py.def("set_object_pose",(void (VrepInterface::*) (const int&, const int&, const DQ&, const VrepInterface::OP_MODES&) const)&VrepInterface::set_object_pose,"Sets object pose.");
    vrepinterface_py.def("set_object_pose",(void (VrepInterface::*) (const std::string&, const int&, const DQ&,  const VrepInterface::OP_MODES&))&VrepInterface::set_object_pose,"Sets object pose.");
    vrepinterface_py.def("set_object_pose",(void (VrepInterface::*) (const int&, const std::string&, const DQ&,  const VrepInterface::OP_MODES&))&VrepInterface::set_object_pose,"Sets object pose.");
    vrepinterface_py.def("set_object_pose",(void (VrepInterface::*) (const std::string&, const std::string&, const DQ&,  const VrepInterface::OP_MODES&))&VrepInterface::set_object_pose,"Sets object pose.");

    vrepinterface_py.def("get_object_poses",&VrepInterface::get_object_poses,"Get the poses of many objects");
    vrepinterface_py.def("set_object_poses",&VrepInterface::set_object_poses,"Set object poses of many objects");

    vrepinterface_py.def("set_joint_position",(void (VrepInterface::*) (const int&, const double&, const VrepInterface::OP_MODES&) const)  &VrepInterface::set_joint_position,"Set joint position");
    vrepinterface_py.def("set_joint_position",(void (VrepInterface::*) (const std::string&, const double&, const VrepInterface::OP_MODES&))&VrepInterface::set_joint_position,"Set joint position");

    vrepinterface_py.def("set_joint_target_position",(void (VrepInterface::*) (const int&, const double&, const VrepInterface::OP_MODES&) const)  &VrepInterface::set_joint_target_position,"Set joint position");
    vrepinterface_py.def("set_joint_target_position",(void (VrepInterface::*) (const std::string&, const double&, const VrepInterface::OP_MODES&))&VrepInterface::set_joint_target_position,"Set joint position");

    vrepinterface_py.def("get_joint_position",(double (VrepInterface::*) (const int&, const VrepInterface::OP_MODES&) const)  &VrepInterface::get_joint_position,"Get joint position");
    vrepinterface_py.def("get_joint_position",(double (VrepInterface::*) (const std::string&, const VrepInterface::OP_MODES&))&VrepInterface::get_joint_position,"Get joint position");

    vrepinterface_py.def("set_joint_positions",(void (VrepInterface::*) (const std::vector<int>&, const VectorXd&, const VrepInterface::OP_MODES&) const)  &VrepInterface::set_joint_positions,"Set joint positions");
    vrepinterface_py.def("set_joint_positions",(void (VrepInterface::*) (const std::vector<std::string>&, const VectorXd&, const VrepInterface::OP_MODES&))&VrepInterface::set_joint_positions,"Set joint positions");

    vrepinterface_py.def("set_joint_target_positions",(void (VrepInterface::*) (const std::vector<int>&, const VectorXd&, const VrepInterface::OP_MODES&) const)  &VrepInterface::set_joint_target_positions,"Set joint positions");
    vrepinterface_py.def("set_joint_target_positions",(void (VrepInterface::*) (const std::vector<std::string>&, const VectorXd&, const VrepInterface::OP_MODES&))&VrepInterface::set_joint_target_positions,"Set joint positions");

    vrepinterface_py.def("get_joint_positions",(VectorXd (VrepInterface::*) (const std::vector<int>&, const VrepInterface::OP_MODES&) const)  &VrepInterface::get_joint_positions,"Get joint positions");
    vrepinterface_py.def("get_joint_positions",(VectorXd (VrepInterface::*) (const std::vector<std::string>&, const VrepInterface::OP_MODES&))&VrepInterface::get_joint_positions,"Get joint positions");

}

