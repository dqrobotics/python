#include "../dqrobotics_module.h"

void init_DQ_SerialManipulator_py(py::module& m)
{
    /***************************************************
    *  DQ SerialManipulator
    * **************************************************/
    py::class_<DQ_SerialManipulator,DQ_Kinematics> dqserialmanipulator_py(m, "DQ_SerialManipulator");
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
}
