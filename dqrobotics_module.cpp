#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

#include <dqrobotics/DQ.h>
#include <dqrobotics/DQ_kinematics.h>

#include <dqrobotics/A2arm.h>
#include <dqrobotics/AX18.h>
#include <dqrobotics/Comau.h>
#include <dqrobotics/Kukka.h>
#include <dqrobotics/Schunk.h>
#include <dqrobotics/WAM.h>
#include <dqrobotics/WAM4.h>

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
    dq.def("P"                  ,&DQ::P,"Retrieves the primary part of a DQ.");
    dq.def("D"                  ,&DQ::D,"Retrieves the dual part of a DQ.");
    dq.def("Re"                 ,&DQ::Re,"Retrieves the real part of a DQ.");
    dq.def("Im"                 ,&DQ::Im,"Retrieves the imaginary part of a DQ.");
    dq.def("conj"               ,&DQ::conj,"Retrieves the conjugate of a DQ.");
    dq.def("norm"               ,&DQ::norm,"Retrieves the norm of a DQ.");
    dq.def("inv"                ,&DQ::inv, "Retrieves the inverse of a DQ.");
    dq.def("translation"        ,&DQ::translation,"Retrieves the translation represented by a unit DQ.");
    dq.def("rotation_axis"      ,&DQ::rotation_axis,"Retrieves the rotation axis represented by a unit DQ.");
    dq.def("rotation_angle"     ,&DQ::rotation_angle,"Retrieves the rotation angle represented by a unit DQ.");
    dq.def("log"                ,&DQ::log,"Retrieves the logarithm of a DQ.");
    dq.def("exp"                ,&DQ::exp,"Retrieves the exp of a DQ.");
    dq.def("pow"                ,&DQ::pow,"Retrieves the pow of a DQ.");
    dq.def("tplus"              ,&DQ::tplus,"Retrieves the tplus operators for a DQ.");
    dq.def("pinv"               ,&DQ::pinv , "Retrieves the pinv of a DQ.");
    //NOT DEFINED IN CPP dq.def("dec_mult"           ,&DQ::dec_mult,"Retrieves the dec mult of a DQ.");
    dq.def("hamiplus4"          ,&DQ::hamiplus4,"Retrieves the H+ operator for the primary part of a DQ.");
    dq.def("haminus4"           ,&DQ::haminus4,"Retrieves the H- operator for the primary part of a DQ.");
    dq.def("hamiplus8"          ,&DQ::hamiplus8,"Retrieves the H+ operator for a DQ.");
    dq.def("haminus8"           ,&DQ::haminus8,"Retrieves the H- operator for a DQ.");
    dq.def("vec4"               ,&DQ::vec4,"Retrieves the primary part of a DQ as a vector.");
    dq.def("vec8"               ,&DQ::vec8,"Retrieves the DQ as a vector.");
    dq.def("generalizedJacobian",&DQ::generalizedJacobian,"Retrieves the generalized Jacobian of a DQ.");
    dq.def("normalize"          ,&DQ::normalize,"Returns a normalized DQ.");

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
    m.def("C8"                 ,&DQ_robotics::C8,"Returns the C8 matrix.");
    m.def("C4"                 ,&DQ_robotics::C4,"Returns the C4 matrix.");
    m.def("P"                  ,&DQ_robotics::P,"Retrieves the primary part of a DQ.");
    m.def("D"                  ,&DQ_robotics::D,"Retrieves the dual part of a DQ.");
    m.def("Re"                 ,&DQ_robotics::Re,"Retrieves the real part of a DQ.");
    m.def("Im"                 ,&DQ_robotics::Im,"Retrieves the imaginary part of a DQ.");
    m.def("conj"               ,&DQ_robotics::conj,"Retrieves the conjugate of a DQ.");
    m.def("norm"               ,&DQ_robotics::norm,"Retrieves the norm of a DQ.");
    m.def("inv"                ,&DQ_robotics::inv, "Retrieves the inverse of a DQ.");
    m.def("translation"        ,&DQ_robotics::translation,"Retrieves the translation represented by a unit DQ.");
    m.def("rotation_axis"      ,&DQ_robotics::rotation_axis,"Retrieves the rotation axis represented by a unit DQ.");
    m.def("rotation_angle"     ,&DQ_robotics::rotation_angle,"Retrieves the rotation angle represented by a unit DQ.");
    m.def("log"                ,&DQ_robotics::log,"Retrieves the logarithm of a DQ.");
    m.def("exp"                ,&DQ_robotics::exp,"Retrieves the exp of a DQ.");
    //m.def("pow"                ,&DQ_robotics::pow,"Retrieves the pow of a DQ.");
    m.def("tplus"              ,&DQ_robotics::tplus,"Retrieves the tplus operators for a DQ.");
    m.def("pinv"               ,&DQ_robotics::pinv , "Retrieves the pinv of a DQ.");
    m.def("dec_mult"           ,&DQ_robotics::dec_mult,"Retrieves the dec mult of a DQ.");
    m.def("hamiplus4"          ,&DQ_robotics::hamiplus4,"Retrieves the H+ operator for the primary part of a DQ.");
    m.def("haminus4"           ,&DQ_robotics::haminus4,"Retrieves the H- operator for the primary part of a DQ.");
    m.def("hamiplus8"          ,&DQ_robotics::hamiplus8,"Retrieves the H+ operator for a DQ.");
    m.def("haminus8"           ,&DQ_robotics::haminus8,"Retrieves the H- operator for a DQ.");
    m.def("vec4"               ,&DQ_robotics::vec4,"Retrieves the primary part of a DQ as a vector.");
    m.def("vec8"               ,&DQ_robotics::vec8,"Retrieves the DQ as a vector.");
    m.def("generalizedJacobian",&DQ_robotics::generalizedJacobian,"Retrieves the generalized Jacobian of a DQ.");
    m.def("normalize"          ,&DQ_robotics::normalize,"Returns a normalized DQ.");
    ///Namespace readonly
    m.attr("DQ_threshold") = DQ_threshold;

    /*****************************************************
     *  DQ Kinematics
     * **************************************************/
    py::class_<DQ_kinematics> dqkinematics(m, "DQ_kinematics");
    dqkinematics.def(py::init<MatrixXd, std::string>());
    dqkinematics.def(py::init());
    ///Methods
    dqkinematics.def("getDHMatrix",       &DQ_kinematics::getDHMatrix,"Gets the DH matrix.");
    dqkinematics.def("links",             &DQ_kinematics::links,"Retrieves the link count.");
    dqkinematics.def("theta",             &DQ_kinematics::theta,"Retrieves the vector of thetas.");
    dqkinematics.def("d",                 &DQ_kinematics::d,"Retrieves the vector d.");
    dqkinematics.def("a",                 &DQ_kinematics::a,"Retrieves the vector a.");
    dqkinematics.def("alpha",             &DQ_kinematics::alpha,"Retrieves the vector of alphas.");
    dqkinematics.def("dummy",             &DQ_kinematics::dummy,"Retrieves the vector of dummies.");
    dqkinematics.def("set_dummy",         &DQ_kinematics::set_dummy,"Sets the vector of dummies.");
    dqkinematics.def("n_dummy",           &DQ_kinematics::n_dummy,"Retrieves the number of dummy joints.");
    dqkinematics.def("convention",        &DQ_kinematics::convention,"Retrieves the DH convention.");
    dqkinematics.def("base",              &DQ_kinematics::base,"Retrieves the base.");
    dqkinematics.def("effector",          &DQ_kinematics::effector,"Retrieves the effector.");
    dqkinematics.def("set_base",          &DQ_kinematics::set_base,"Sets the base.");
    dqkinematics.def("set_effector",      &DQ_kinematics::set_effector,"Sets the effector.");
    dqkinematics.def("raw_fkm",           (DQ (DQ_kinematics::*)(const VectorXd&) const)&DQ_kinematics::raw_fkm,"Gets the raw fkm.");
    dqkinematics.def("raw_fkm",           (DQ (DQ_kinematics::*)(const VectorXd&,const int&) const)&DQ_kinematics::raw_fkm,"Gets the raw fkm.");
    dqkinematics.def("fkm",               (DQ (DQ_kinematics::*)(const VectorXd&) const)&DQ_kinematics::fkm,"Gets the fkm.");
    dqkinematics.def("fkm",               (DQ (DQ_kinematics::*)(const VectorXd&,const int&) const)&DQ_kinematics::fkm,"Gets the fkm.");
    dqkinematics.def("dh2dq",             &DQ_kinematics::dh2dq,"Returns a link's DH transformation as a DQ.");
    dqkinematics.def("get_z",             &DQ_kinematics::get_z,"Returns the z of a transformation.");
    dqkinematics.def("analyticalJacobian",&DQ_kinematics::analyticalJacobian,"Returns the analytical Jacobian");

    ///Namespace Functions
    m.def("links",&DQ_robotics::links,"Retrieves the link count.");
    m.def("theta",&DQ_robotics::theta,"Retrieves the vector of thetas.");
    m.def("d",&DQ_robotics::d,"Retrieves the vector d.");
    m.def("a",&DQ_robotics::a,"Retrieves the vector a.");
    m.def("alpha",&DQ_robotics::alpha,"Retrieves the vector of alphas.");
    m.def("dummy",&DQ_robotics::dummy,"Retrieves the vector of dummies.");
    //NOT IN CPP m.def("set_dummy",&DQ_robotics::set_dummy,"Sets the vector of dummies.");
    m.def("n_dummy",&DQ_robotics::n_dummy,"Retrieves the number of dummy joints.");
    m.def("convention",&DQ_robotics::convention,"Retrieves the DH convention.");
    m.def("base",&DQ_robotics::base,"Retrieves the base.");
    m.def("effector",&DQ_robotics::effector,"Retrieves the effector.");
    m.def("set_base",&DQ_robotics::set_base,"Sets the base.");
    m.def("set_effector",&DQ_robotics::set_effector,"Sets the effector.");
    m.def("raw_fkm",(DQ (*)(const DQ_kinematics&, const VectorXd&))DQ_robotics::raw_fkm ,"Gets the raw fkm.");
    m.def("raw_fkm",(DQ (*)(const DQ_kinematics&, const VectorXd&, const int&))DQ_robotics::raw_fkm ,"Gets the raw fkm.");
    //NOT IN CPP m.def("fkm",(DQ (*)(const DQ_kinematics&, const VectorXd&))DQ_robotics::fkm ,"Gets the fkm.");
    //NOT IN CPP m.def("fkm",(DQ (*)(const DQ_kinematics&, const VectorXd&, const int&))DQ_robotics::fkm ,"Gets the fkm.");

    m.def("dh2dq",&DQ_robotics::dh2dq,"Returns a link's DH transformation as a DQ.");
    m.def("get_z",&DQ_robotics::get_z,"Returns the z of a transformation.");
    m.def("analyticalJacobian",&DQ_robotics::analyticalJacobian,"Returns the analytical Jacobian");
    m.def("rotationJacobian",  &DQ_robotics::rotationJacobian,"Returns the rotation Jacobian");
    //m.def("distanceJacobian",  &DQ_robotics::distanceJacobian,"Returns the distance Jacobian");
    m.def("pseudoInverse",     &DQ_robotics::pseudoInverse,"Returns the pseudoinverse of a matrix");

    /*****************************************************
     *  Robots Kinematic Models
     * **************************************************/
    py::module robot_dh = m.def_submodule("robot_dh", "A submodule of dqrobotics");
    //#include <dqrobotics/A2arm.h>
    robot_dh.def("A2armKinematics",&A2armKinematics,"Returns the A2 arm kinematics.");
    //#include <dqrobotics/AX18.h>
    robot_dh.def("Ax18armKinematics",&Ax18Kinematics,"Returns the Ax18 arm kinematics.");
    //#include <dqrobotics/Comau.h>
    robot_dh.def("ComauKinematics",&ComauKinematics,"Returns the Comau arm kinematics.");
    //#include <dqrobotics/Kukka.h>
    robot_dh.def("KukkaKinematics",&KukkaKinematics,"Returns the Kukka arm kinematics.");
    //#include <dqrobotics/Schunk.h>
    robot_dh.def("SchunkKinematics",&SchunkKinematics,"Returns the Ax18 arm kinematics.");
    //#include <dqrobotics/WAM.h>
    robot_dh.def("WamKinematics",&WamKinematics,"Returns the Wam arm kinematics.");
    //#include <dqrobotics/WAM4.h>
    robot_dh.def("Wam4Kinematics",&Wam4Kinematics,"Returns the Wam4 arm kinematics.");
}

