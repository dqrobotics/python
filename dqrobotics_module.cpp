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

#include <dqrobotics/robot_control/DQ_KinematicController.h>
#include <dqrobotics/robot_control/DQ_KinematicConstrainedController.h>
#include <dqrobotics/robot_control/DQ_TaskSpacePseudoInverseController.h>
#include <dqrobotics/robot_control/DQ_TaskspaceQuadraticProgrammingController.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

#include <dqrobotics/solvers/DQ_QuadraticProgrammingSolver.h>

#include <dqrobotics/robots/Ax18ManipulatorRobot.h>
#include <dqrobotics/robots/BarrettWamArmRobot.h>
#include <dqrobotics/robots/ComauSmartSixRobot.h>
#include <dqrobotics/robots/KukaLw4Robot.h>

#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepRobot.h>
#include <dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.h>
#include <dqrobotics/interfaces/vrep/robots/YouBotVrepRobot.h>

using namespace DQ_robotics;
using namespace Eigen;

//https://pybind11.readthedocs.io/en/stable/advanced/classes.html
class DQ_QuadraticProgrammingSolverPy : public DQ_QuadraticProgrammingSolver
{
public:
    /* Inherit the constructors */
    using DQ_QuadraticProgrammingSolver::DQ_QuadraticProgrammingSolver;

    /* Trampoline (need one for each virtual function) */
    VectorXd solve_quadratic_program(const MatrixXd &H, const MatrixXd &f, const MatrixXd A, const MatrixXd &b, const MatrixXd &Aeq, const MatrixXd &beq) override{
        PYBIND11_OVERLOAD_PURE(
                    VectorXd,                       /* Return type */
                    DQ_QuadraticProgrammingSolver,  /* Parent class */
                    solve_quadratic_program,        /* Name of function in C++ (must match Python name) */
                    H, f, A, b, Aeq, beq            /* Argument(s) */
                    )
    }
};

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

    ///Operators
    //Self
    dq.def(py::self + py::self);
    dq.def(py::self * py::self);
    dq.def(py::self - py::self);
    dq.def(py::self == py::self);
    dq.def(py::self != py::self);
    dq.def(- py::self);
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

    ///Namespace readonly
    m.attr("DQ_threshold") = DQ_threshold;
    m.attr("i_")           = DQ_robotics::i_;
    m.attr("j_")           = DQ_robotics::j_;
    m.attr("k_")           = DQ_robotics::k_;
    m.attr("E_")           = DQ_robotics::E_;

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
    geometry_py.def_static("line_to_line_angle",               &DQ_Geometry::line_to_line_angle,              "Returns the angle between two lines");

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
    ///Methods
    dqserialmanipulator_py.def("getDHMatrix",                 &DQ_SerialManipulator::getDHMatrix,"Gets the DH matrix.");
    dqserialmanipulator_py.def("theta",                       &DQ_SerialManipulator::theta,"Retrieves the vector of thetas.");
    dqserialmanipulator_py.def("d",                           &DQ_SerialManipulator::d,"Retrieves the vector d.");
    dqserialmanipulator_py.def("a",                           &DQ_SerialManipulator::a,"Retrieves the vector a.");
    dqserialmanipulator_py.def("alpha",                       &DQ_SerialManipulator::alpha,"Retrieves the vector of alphas.");
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
    dqwholebody_py.def(py::init<std::shared_ptr<DQ_Kinematics>>());
    dqwholebody_py.def("add",&DQ_WholeBody::add,"Adds a DQ_Kinematics pointer to the kinematic chain.");
    dqwholebody_py.def("fkm",(DQ (DQ_WholeBody::*)(const VectorXd&) const)&DQ_WholeBody::fkm,"Gets the fkm.");
    dqwholebody_py.def("fkm",(DQ (DQ_WholeBody::*)(const VectorXd&,const int&) const)&DQ_WholeBody::fkm,"Gets the fkm.");
    dqwholebody_py.def("get_dim_configuration_space",&DQ_WholeBody::get_dim_configuration_space,"Gets the dimention of the configuration space");
    dqwholebody_py.def("get_chain",&DQ_WholeBody::get_chain, "Returns the DQ_Kinematics at a given index of the chain");
    dqwholebody_py.def("get_chain_as_serial_manipulator",&DQ_WholeBody::get_chain_as_serial_manipulator, "Returns the DQ_SerialManipulator at a given index of the chain");
    dqwholebody_py.def("get_chain_as_holonomic_base",&DQ_WholeBody::get_chain_as_holonomic_base, "Returns the DQ_HolonomicBase at a given index of the chain");
    dqwholebody_py.def("pose_jacobian",&DQ_WholeBody::pose_jacobian,"Returns the combined pose Jacobian");

    /*****************************************************
     *  Solvers <dqrobotics/solvers/...>
     * **************************************************/
    py::module solvers = m.def_submodule("solvers", "The solvers submodule of dqrobotics");

    /*****************************************************
     *  DQ DQ_QuadraticProgrammingSolver
     * **************************************************/
    py::class_<DQ_QuadraticProgrammingSolver, DQ_QuadraticProgrammingSolverPy> dqquadraticprogrammingsolver_py(solvers,"DQ_QuadraticProgrammingSolver");
    dqquadraticprogrammingsolver_py.def("solve_quadratic_program", &DQ_QuadraticProgrammingSolver::solve_quadratic_program, "Solvers a quadratic program");

    /*****************************************************
     *  Robot Control <dqrobotics/robot_control/...>
     * **************************************************/
    py::module robot_control = m.def_submodule("robot_control", "The robot_control submodule of dqrobotics");

    py::enum_<ControlObjective>(robot_control, "ControlObjective")
            .value("Line",           ControlObjective::Line)
            .value("None",           ControlObjective::None)
            .value("Pose",           ControlObjective::Pose)
            .value("Plane",          ControlObjective::Plane)
            .value("Distance",       ControlObjective::Distance)
            .value("Rotation",       ControlObjective::Rotation)
            .value("Translation",    ControlObjective::Translation)
            .export_values();

    /*****************************************************
     *  DQ KinematicController
     * **************************************************/
    py::class_<DQ_KinematicController> dqkinematiccontroller_py(robot_control,"DQ_KinematicController");
    dqkinematiccontroller_py.def("get_control_objective"  ,&DQ_KinematicController::get_control_objective,"Gets the control objective");
    dqkinematiccontroller_py.def("get_jacobian"           ,&DQ_KinematicController::get_jacobian,"Gets the Jacobian");
    dqkinematiccontroller_py.def("get_last_error_signal"  ,&DQ_KinematicController::get_last_error_signal, "Gets the last error signal");
    dqkinematiccontroller_py.def("is_set"                 ,&DQ_KinematicController::is_set,"Checks if the controller's objective has been set");
    dqkinematiccontroller_py.def("is_stable"              ,&DQ_KinematicController::is_stable,"Checks if the controller has stabilized");
    dqkinematiccontroller_py.def("set_control_objective"  ,&DQ_KinematicController::set_control_objective,"Sets the control objective");
    dqkinematiccontroller_py.def("set_gain"               ,&DQ_KinematicController::set_gain,"Sets the controller gain");
    dqkinematiccontroller_py.def("set_stability_threshold",&DQ_KinematicController::set_stability_threshold,"Sets the stability threshold");
    dqkinematiccontroller_py.def("set_damping"            ,&DQ_KinematicController::set_damping, "Sets the damping.");

    /*****************************************************
     *  DQ TaskSpacePseudoInverseController
     * **************************************************/
    py::class_<DQ_TaskSpacePseudoInverseController, DQ_KinematicController> dqtaskspacepseudoinversecontroller_py(robot_control,"DQ_TaskSpacePseudoInverseController");
    dqtaskspacepseudoinversecontroller_py.def(py::init<DQ_Kinematics*>());
    dqtaskspacepseudoinversecontroller_py.def("compute_setpoint_control_signal",&DQ_TaskSpacePseudoInverseController::compute_setpoint_control_signal,"Computes the setpoint control signal.");
    dqtaskspacepseudoinversecontroller_py.def("compute_tracking_control_signal",&DQ_TaskSpacePseudoInverseController::compute_tracking_control_signal,"Computes the tracking control signal.");

    /*****************************************************
     *  DQ KinematicConstrainedController
     * **************************************************/
    py::class_<DQ_KinematicConstrainedController, DQ_KinematicController> dqkinematicconstrainedcontroller_py(robot_control,"DQ_KinematicConstrainedController");
    dqkinematicconstrainedcontroller_py.def("set_equality_constraint", &DQ_KinematicConstrainedController::set_equality_constraint,  "Sets equality constraints.");
    dqkinematicconstrainedcontroller_py.def("set_inequality_constraint", &DQ_KinematicConstrainedController::set_inequality_constraint,  "Sets inequality constraints.");

    /*****************************************************
     *  DQ TaskspaceQuadraticProgrammingController
     * **************************************************/
    py::class_<DQ_TaskspaceQuadraticProgrammingController, DQ_KinematicConstrainedController> dqtaskspacequadraticprogrammingcontroller_py(robot_control,"DQ_TaskspaceQuadraticProgrammingController");
    dqtaskspacequadraticprogrammingcontroller_py.def("compute_objective_function_symmetric_matrix", &DQ_TaskspaceQuadraticProgrammingController::compute_objective_function_symmetric_matrix, "Compute symmetric matrix.");
    dqtaskspacequadraticprogrammingcontroller_py.def("compute_objective_function_linear_component", &DQ_TaskspaceQuadraticProgrammingController::compute_objective_function_linear_component, "Compute the objective function.");
    dqtaskspacequadraticprogrammingcontroller_py.def("compute_setpoint_control_signal", &DQ_TaskspaceQuadraticProgrammingController::compute_setpoint_control_signal, "Compute the setpoint control signal.");
    dqtaskspacequadraticprogrammingcontroller_py.def("compute_tracking_control_signal", &DQ_TaskspaceQuadraticProgrammingController::compute_tracking_control_signal, "Compute the tracking control signal.");

    /*****************************************************
     *  DQ ClassicQPController
     * **************************************************/
    py::class_<DQ_ClassicQPController, DQ_TaskspaceQuadraticProgrammingController> dq_classicqpcontroller_py(robot_control,"DQ_ClassicQPController");
    dq_classicqpcontroller_py.def(py::init<DQ_Kinematics*, DQ_QuadraticProgrammingSolver*>());
    dq_classicqpcontroller_py.def("compute_objective_function_symmetric_matrix", &DQ_ClassicQPController::compute_objective_function_symmetric_matrix, "Compute symmetric matrix.");
    dq_classicqpcontroller_py.def("compute_objective_function_linear_component", &DQ_ClassicQPController::compute_objective_function_linear_component, "Compute the objective function.");

    /*****************************************************
     *  Interfaces Submodule
     * **************************************************/
    py::module interfaces_py = m.def_submodule("interfaces", "A submodule of dqrobotics");

    /*****************************************************
     *  Vrep Submodule
     * **************************************************/
    py::module vrep_py = interfaces_py.def_submodule("vrep", "A submodule of dqrobotics");
    vrep_py.attr("VREP_OBJECTNAME_ABSOLUTE") = VREP_OBJECTNAME_ABSOLUTE;

    /*****************************************************
     *  VrepInterface
     * **************************************************/
    py::class_<DQ_VrepInterface> dqvrepinterface_py(vrep_py,"DQ_VrepInterface");
    dqvrepinterface_py.def(py::init<>());
    dqvrepinterface_py.def(py::init<std::atomic_bool*>());

    py::enum_<DQ_VrepInterface::OP_MODES>(dqvrepinterface_py, "OP_MODES")
            .value("OP_BUFFER",    DQ_VrepInterface::OP_MODES::OP_BUFFER)
            .value("OP_ONESHOT",   DQ_VrepInterface::OP_MODES::OP_ONESHOT)
            .value("OP_BLOCKING",  DQ_VrepInterface::OP_MODES::OP_BLOCKING)
            .value("OP_STREAMING", DQ_VrepInterface::OP_MODES::OP_STREAMING)
            .value("OP_AUTOMATIC", DQ_VrepInterface::OP_MODES::OP_AUTOMATIC)
            .export_values();

    dqvrepinterface_py.def("connect",(bool (DQ_VrepInterface::*) (const int&, const int&, const int&))&DQ_VrepInterface::connect,"Connects to V-REP in local machine.");
    dqvrepinterface_py.def("connect",(bool (DQ_VrepInterface::*) (const std::string&, const int&, const int&, const int&))&DQ_VrepInterface::connect,"Connects to V-REP with a given ip.");

    dqvrepinterface_py.def("disconnect",    &DQ_VrepInterface::disconnect,"Disconnects from V-REP.");
    dqvrepinterface_py.def("disconnect_all",&DQ_VrepInterface::disconnect_all,"Disconnect all from V-REP");

    dqvrepinterface_py.def("start_simulation",&DQ_VrepInterface::start_simulation,"Start simulation");
    dqvrepinterface_py.def("stop_simulation", &DQ_VrepInterface::stop_simulation,"Stops simulation");

    dqvrepinterface_py.def("is_simulation_running",&DQ_VrepInterface::is_simulation_running,"Checks whether the simulation is running or not");

    dqvrepinterface_py.def("get_object_handle", &DQ_VrepInterface::get_object_handle,"Gets an object handle");
    dqvrepinterface_py.def("get_object_handles",&DQ_VrepInterface::get_object_handles,"Get object handles");

    dqvrepinterface_py.def("get_object_translation",(DQ (DQ_VrepInterface::*) (const int&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_translation,"Gets object translation.");
    dqvrepinterface_py.def("get_object_translation",(DQ (DQ_VrepInterface::*) (const std::string&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_translation,"Gets object translation.");
    dqvrepinterface_py.def("get_object_translation",(DQ (DQ_VrepInterface::*) (const int&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_translation,"Gets object translation.");
    dqvrepinterface_py.def("get_object_translation",(DQ (DQ_VrepInterface::*) (const std::string&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_translation,"Gets object translation.");

    dqvrepinterface_py.def("set_object_translation",(void (DQ_VrepInterface::*) (const int&, const int&, const DQ&, const DQ_VrepInterface::OP_MODES&) const)&DQ_VrepInterface::set_object_translation,"Sets object translation.");
    dqvrepinterface_py.def("set_object_translation",(void (DQ_VrepInterface::*) (const std::string&, const int&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_translation,"Sets object translation.");
    dqvrepinterface_py.def("set_object_translation",(void (DQ_VrepInterface::*) (const int&, const std::string&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_translation,"Sets object translation.");
    dqvrepinterface_py.def("set_object_translation",(void (DQ_VrepInterface::*) (const std::string&, const DQ&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_translation,"Sets object translation.");

    dqvrepinterface_py.def("get_object_rotation",(DQ (DQ_VrepInterface::*) (const int&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_rotation,"Gets object rotation.");
    dqvrepinterface_py.def("get_object_rotation",(DQ (DQ_VrepInterface::*) (const std::string&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_rotation,"Gets object rotation.");
    dqvrepinterface_py.def("get_object_rotation",(DQ (DQ_VrepInterface::*) (const int&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_rotation,"Gets object rotation.");
    dqvrepinterface_py.def("get_object_rotation",(DQ (DQ_VrepInterface::*) (const std::string&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_rotation,"Gets object rotation.");

    dqvrepinterface_py.def("set_object_rotation",(void (DQ_VrepInterface::*) (const int&, const int&, const DQ&, const DQ_VrepInterface::OP_MODES&) const)&DQ_VrepInterface::set_object_rotation,"Sets object rotation.");
    dqvrepinterface_py.def("set_object_rotation",(void (DQ_VrepInterface::*) (const std::string&, const int&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_rotation,"Sets object rotation.");
    dqvrepinterface_py.def("set_object_rotation",(void (DQ_VrepInterface::*) (const int&, const std::string&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_rotation,"Sets object rotation.");
    dqvrepinterface_py.def("set_object_rotation",(void (DQ_VrepInterface::*) (const std::string&, const DQ&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_rotation,"Sets object rotation.");

    dqvrepinterface_py.def("get_object_pose",(DQ (DQ_VrepInterface::*) (const int&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_pose,"Gets object pose.");
    dqvrepinterface_py.def("get_object_pose",(DQ (DQ_VrepInterface::*) (const std::string&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_pose,"Gets object pose.");
    dqvrepinterface_py.def("get_object_pose",(DQ (DQ_VrepInterface::*) (const int&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_pose,"Gets object pose.");
    dqvrepinterface_py.def("get_object_pose",(DQ (DQ_VrepInterface::*) (const std::string&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_pose,"Gets object pose.");

    dqvrepinterface_py.def("set_object_pose",(void (DQ_VrepInterface::*) (const int&, const int&, const DQ&, const DQ_VrepInterface::OP_MODES&) const)&DQ_VrepInterface::set_object_pose,"Sets object pose.");
    dqvrepinterface_py.def("set_object_pose",(void (DQ_VrepInterface::*) (const std::string&, const int&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_pose,"Sets object pose.");
    dqvrepinterface_py.def("set_object_pose",(void (DQ_VrepInterface::*) (const int&, const std::string&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_pose,"Sets object pose.");
    dqvrepinterface_py.def("set_object_pose",(void (DQ_VrepInterface::*) (const std::string&, const DQ&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_pose,"Sets object pose.");

    dqvrepinterface_py.def("get_object_poses",&DQ_VrepInterface::get_object_poses,"Get the poses of many objects");
    dqvrepinterface_py.def("set_object_poses",&DQ_VrepInterface::set_object_poses,"Set object poses of many objects");

    dqvrepinterface_py.def("set_joint_position",(void (DQ_VrepInterface::*) (const int&, const double&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::set_joint_position,"Set joint position");
    dqvrepinterface_py.def("set_joint_position",(void (DQ_VrepInterface::*) (const std::string&, const double&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_position,"Set joint position");

    dqvrepinterface_py.def("set_joint_target_position",(void (DQ_VrepInterface::*) (const int&, const double&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::set_joint_target_position,"Set joint position");
    dqvrepinterface_py.def("set_joint_target_position",(void (DQ_VrepInterface::*) (const std::string&, const double&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_target_position,"Set joint position");

    dqvrepinterface_py.def("get_joint_position",(double (DQ_VrepInterface::*) (const int&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::get_joint_position,"Get joint position");
    dqvrepinterface_py.def("get_joint_position",(double (DQ_VrepInterface::*) (const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_joint_position,"Get joint position");

    dqvrepinterface_py.def("set_joint_positions",(void (DQ_VrepInterface::*) (const std::vector<int>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::set_joint_positions,"Set joint positions");
    dqvrepinterface_py.def("set_joint_positions",(void (DQ_VrepInterface::*) (const std::vector<std::string>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_positions,"Set joint positions");

    dqvrepinterface_py.def("set_joint_target_positions",(void (DQ_VrepInterface::*) (const std::vector<int>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::set_joint_target_positions,"Set joint positions");
    dqvrepinterface_py.def("set_joint_target_positions",(void (DQ_VrepInterface::*) (const std::vector<std::string>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_target_positions,"Set joint positions");

    dqvrepinterface_py.def("get_joint_positions",(VectorXd (DQ_VrepInterface::*) (const std::vector<int>&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::get_joint_positions,"Get joint positions");
    dqvrepinterface_py.def("get_joint_positions",(VectorXd (DQ_VrepInterface::*) (const std::vector<std::string>&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_joint_positions,"Get joint positions");

    /*****************************************************
     *  VrepRobot
     * **************************************************/
    py::class_<DQ_VrepRobot> dqvreprobot_py(vrep_py,"DQ_VrepRobot");
    dqvreprobot_py.def("send_q_to_vrep", &DQ_VrepRobot::send_q_to_vrep, "Get joint values from vrep.");
    dqvreprobot_py.def("get_q_from_vrep", &DQ_VrepRobot::get_q_from_vrep, "Send joint values to vrep.");

    /*****************************************************
     *  Vrep Robots Submodule
     * **************************************************/
    py::module vreprobots_py = vrep_py.def_submodule("robots", "A submodule of dqrobotics");

    /*****************************************************
     *  LBR4pVrepRobot
     * **************************************************/
    py::class_<LBR4pVrepRobot,DQ_VrepRobot> lbr4pvreprobot_py(vreprobots_py,"LBR4pVrepRobot");
    lbr4pvreprobot_py.def(py::init<const std::string&, DQ_VrepInterface*>());

    lbr4pvreprobot_py.def("send_q_to_vrep", &LBR4pVrepRobot::send_q_to_vrep, "Send joint values to vrep.");
    lbr4pvreprobot_py.def("get_q_from_vrep", &LBR4pVrepRobot::get_q_from_vrep, "Get joint values from vrep.");
    lbr4pvreprobot_py.def("kinematics", &LBR4pVrepRobot::kinematics, "Get kinematics model.");

    /*****************************************************
     *  YouBotVrepRobot
     * **************************************************/
    py::class_<YouBotVrepRobot,DQ_VrepRobot> youbotvreprobot_py(vreprobots_py,"YouBotVrepRobot");
    youbotvreprobot_py.def(py::init<const std::string&, DQ_VrepInterface*>());

    youbotvreprobot_py.def("send_q_to_vrep", &YouBotVrepRobot::send_q_to_vrep, "Send joint values to vrep.");
    youbotvreprobot_py.def("get_q_from_vrep", &YouBotVrepRobot::get_q_from_vrep, "Get joint values from vrep.");
    youbotvreprobot_py.def("kinematics", &YouBotVrepRobot::kinematics, "Get kinematics model.");

}

