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

#include "dqrobotics_module.h"

/**
 * @brief Binds `DQ`, the class that represents dual quaternions, and the
 * free functions and constants of the `DQ_robotics` namespace declared in
 * `dqrobotics/DQ.h`, to the Python module @p m.
 */
void init_DQ_py(py::module& m)
{
    /*****************************************************
     *  DQ
     * **************************************************/
    py::class_<DQ> dq(m, "DQ",
                       "A dual quaternion, used to represent poses, rotations, translations, "
                       "lines, and planes in three-dimensional space.");
    dq.def(py::init<>(),
           "Constructs a dual quaternion with all coefficients equal to zero.");
    dq.def(py::init<double, double, double, double, double, double, double, double>(),
           py::arg("q0"), py::arg("q1"), py::arg("q2"), py::arg("q3"),
           py::arg("e0"), py::arg("e1"), py::arg("e2"), py::arg("e3"),
           "Constructs a dual quaternion from its eight coefficients, in the order "
           "primary (q0, q1, q2, q3) followed by dual (e0, e1, e2, e3).");
    dq.def(py::init<VectorXd>(), py::arg("v"),
           "Constructs a dual quaternion from a vector of 8, 6, 4, 3, or 1 elements.\n\n"
           "The mapping between the vector size and the resulting dual quaternion "
           "follows the inverse of vec8(), vec6(), vec4(), and vec3(), respectively. "
           "A vector of size 1 is used to construct a real dual quaternion.");
    ///Members
    dq.def_readwrite("q", &DQ::q,
                      "The eight coefficients of this dual quaternion, in the order "
                      "primary (q0, q1, q2, q3) followed by dual (e0, e1, e2, e3).");
    ///Static Members
    dq.def_readonly_static("i",&DQ::i, "The imaginary unit i, such that `i*i = -1`.");
    dq.def_readonly_static("j",&DQ::j, "The imaginary unit j, such that `j*j = -1`.");
    dq.def_readonly_static("k",&DQ::k, "The imaginary unit k, such that `k*k = -1`.");
    dq.def_readonly_static("E",&DQ::E, "The dual unit, such that `E*E = 0`.");
    ///Methods
    dq.def("P"                   ,&DQ::P,                     "Returns the primary part of this dual quaternion.");
    dq.def("D"                   ,&DQ::D,                     "Returns the dual part of this dual quaternion.");
    dq.def("Re"                  ,&DQ::Re,                    "Returns the real part of this dual quaternion.");
    dq.def("Im"                  ,&DQ::Im,                    "Returns the imaginary part of this dual quaternion.");
    dq.def("conj"                ,&DQ::conj,                  "Returns the conjugate of this dual quaternion.");
    dq.def("norm"                ,&DQ::norm,                  "Returns the dual scalar corresponding to the norm of this dual quaternion.");
    dq.def("inv"                 ,&DQ::inv,                   "Returns the inverse of this dual quaternion, given by `conj(this)/(norm(this)^2)`.");
    dq.def("translation"         ,&DQ::translation,           "Returns the translation quaternion of this unit dual quaternion.");
    dq.def("rotation"            ,&DQ::rotation,              "Returns the rotation quaternion of this unit dual quaternion.");
    dq.def("rotation_axis"       ,&DQ::rotation_axis,         "Returns the rotation axis of this unit dual quaternion.");
    dq.def("rotation_angle"      ,&DQ::rotation_angle,        "Returns the rotation angle of this unit dual quaternion.");
    dq.def("log"                 ,&DQ::log,                   "Returns the logarithm of this dual quaternion.");
    dq.def("exp"                 ,&DQ::exp,                   "Returns the exponential of this pure dual quaternion.");
    dq.def("pow"                 ,&DQ::pow,                   py::arg("a"), "Returns this dual quaternion raised to the power of `a`.");
    dq.def("tplus"               ,&DQ::tplus,                 "Returns the unit dual quaternion corresponding to the transformation of this dual quaternion.");
    dq.def("T"                   ,&DQ::T,                     "Alias for tplus().");
    dq.def("pinv"                ,&DQ::pinv ,                 "Returns the Moore-Penrose pseudoinverse of this dual quaternion.");
    dq.def("hamiplus4"           ,&DQ::hamiplus4,             "Returns the Hamilton operator H+ of this dual quaternion, restricted to its primary part.");
    dq.def("haminus4"            ,&DQ::haminus4,              "Returns the Hamilton operator H- of this dual quaternion, restricted to its primary part.");
    dq.def("hamiplus8"           ,&DQ::hamiplus8,             "Returns the Hamilton operator H+ of this dual quaternion.");
    dq.def("haminus8"            ,&DQ::haminus8,              "Returns the Hamilton operator H- of this dual quaternion.");
    dq.def("vec3"                ,&DQ::vec3,                  "Maps the primary part of this dual quaternion into a 3-dimensional vector.");
    dq.def("vec4"                ,&DQ::vec4,                  "Maps the primary part of this dual quaternion into a 4-dimensional vector.");
    dq.def("vec6"                ,&DQ::vec6,                  "Maps this dual quaternion into a 6-dimensional vector, discarding the real part of both the primary and dual components.");
    dq.def("vec8"                ,&DQ::vec8,                  "Maps this dual quaternion into an 8-dimensional vector.");
    dq.def("normalize"           ,&DQ::normalize,             "Returns this dual quaternion normalized to unit norm.");
    dq.def("__repr__"            ,&DQ::to_string,             "Returns a string representation of this dual quaternion, used by Python's print() function.");
    dq.def("to_string"           ,&DQ::to_string,             "Returns a string representation of this dual quaternion.");
    dq.def("generalized_jacobian",&DQ::generalized_jacobian,  "Returns the generalized Jacobian used in the mapping between the time derivative of a unit dual quaternion and the twist it represents.");
    dq.def("sharp"               ,&DQ::sharp,                 "Returns the sharp conjugate of this dual quaternion.");
    dq.def("Ad"                  ,&DQ::Ad,                    py::arg("dq2"), "Returns the adjoint transformation `this * dq2 * this'`.");
    dq.def("Adsharp"             ,&DQ::Adsharp,               py::arg("dq2"), "Returns the sharp adjoint transformation `this.sharp() * dq2 * this'`.");
    dq.def("Q4"                  ,&DQ::Q4,                    "Given the unit quaternion represented by this dual quaternion, returns the partial derivative of that quaternion with respect to its logarithm.");
    dq.def("Q8"                  ,&DQ::Q8,                    "Given this unit dual quaternion, returns the partial derivative of this dual quaternion with respect to its logarithm.");

    ///Operators
    //Self
    dq.def(py::self + py::self, "Returns the dual quaternion addition between this dual quaternion and another.");
    dq.def(py::self * py::self, "Returns the dual quaternion multiplication between this dual quaternion and another.");
    dq.def(py::self - py::self, "Returns the dual quaternion subtraction between this dual quaternion and another.");
    dq.def(py::self == py::self, "Returns true if this dual quaternion and another are equal, up to a numerical threshold.");
    dq.def(py::self != py::self, "Returns true if this dual quaternion and another are different, up to a numerical threshold.");
    dq.def(- py::self, "Returns the additive inverse of this dual quaternion.");
    //Double
    dq.def(double()  * py::self, "Returns the multiplication between a scalar and this dual quaternion.");
    dq.def(py::self * double(), "Returns the multiplication between this dual quaternion and a scalar.");
    dq.def(double()  + py::self, "Returns the addition between a scalar and this dual quaternion.");
    dq.def(py::self + double(), "Returns the addition between this dual quaternion and a scalar.");
    dq.def(double()  - py::self, "Returns the subtraction between a scalar and this dual quaternion.");
    dq.def(py::self - double(), "Returns the subtraction between this dual quaternion and a scalar.");
    dq.def(double()  == py::self, "Returns true if a scalar and this dual quaternion are equal, up to a numerical threshold.");
    dq.def(py::self == double(), "Returns true if this dual quaternion and a scalar are equal, up to a numerical threshold.");
    dq.def(double()  != py::self, "Returns true if a scalar and this dual quaternion are different, up to a numerical threshold.");
    dq.def(py::self != double(), "Returns true if this dual quaternion and a scalar are different, up to a numerical threshold.");

    ///Namespace Functions
    m.def("C8"                  ,&DQ_robotics::C8,                   "Returns the conjugator matrix associated with vec8().");
    m.def("C4"                  ,&DQ_robotics::C4,                   "Returns the conjugator matrix associated with vec4().");
    m.def("P"                   ,&DQ_robotics::P,                    py::arg("dq"), "Returns the primary part of `dq`.");
    m.def("D"                   ,&DQ_robotics::D,                    py::arg("dq"), "Returns the dual part of `dq`.");
    m.def("Re"                  ,&DQ_robotics::Re,                   py::arg("dq"), "Returns the real part of `dq`.");
    m.def("Im"                  ,&DQ_robotics::Im,                   py::arg("dq"), "Returns the imaginary part of `dq`.");
    m.def("conj"                ,&DQ_robotics::conj,                 py::arg("dq"), "Returns the conjugate of `dq`.");
    m.def("norm"                ,&DQ_robotics::norm,                 py::arg("dq"), "Returns the dual scalar corresponding to the norm of `dq`.");
    m.def("inv"                 ,&DQ_robotics::inv,                  py::arg("dq"), "Returns the inverse of `dq`, given by `conj(dq)/(norm(dq)^2)`.");
    m.def("translation"         ,&DQ_robotics::translation,          py::arg("dq"), "Returns the translation quaternion of the unit dual quaternion `dq`.");
    m.def("rotation"            ,&DQ_robotics::rotation,             py::arg("dq"), "Returns the rotation quaternion of the unit dual quaternion `dq`.");
    m.def("rotation_axis"       ,&DQ_robotics::rotation_axis,        py::arg("dq"), "Returns the rotation axis of the unit dual quaternion `dq`.");
    m.def("rotation_angle"      ,&DQ_robotics::rotation_angle,       py::arg("dq"), "Returns the rotation angle of the unit dual quaternion `dq`.");
    m.def("log"                 ,&DQ_robotics::log,                  py::arg("dq"), "Returns the logarithm of `dq`.");
    m.def("exp"                 ,&DQ_robotics::exp,                  py::arg("dq"), "Returns the exponential of the pure dual quaternion `dq`.");
    m.def("pow"                 ,&DQ_robotics::pow,                  py::arg("dq"), py::arg("a"), "Returns `dq` raised to the power of `a`.");
    m.def("tplus"               ,&DQ_robotics::tplus,                py::arg("dq"), "Returns the unit dual quaternion corresponding to the transformation of `dq`.");
    m.def("pinv"                ,(DQ (*) (const DQ&)) &DQ_robotics::pinv , py::arg("dq"), "Returns the Moore-Penrose pseudoinverse of `dq`.");
    m.def("dec_mult"            ,&DQ_robotics::dec_mult,             py::arg("dq1"), py::arg("dq2"), "Returns the decompositional multiplication between `dq1` and `dq2`.");
    m.def("hamiplus4"           ,&DQ_robotics::hamiplus4,            py::arg("dq"), "Returns the Hamilton operator H+ of `dq`, restricted to its primary part.");
    m.def("haminus4"            ,&DQ_robotics::haminus4,             py::arg("dq"), "Returns the Hamilton operator H- of `dq`, restricted to its primary part.");
    m.def("hamiplus8"           ,&DQ_robotics::hamiplus8,            py::arg("dq"), "Returns the Hamilton operator H+ of `dq`.");
    m.def("haminus8"            ,&DQ_robotics::haminus8,             py::arg("dq"), "Returns the Hamilton operator H- of `dq`.");
    m.def("vec3"                ,&DQ_robotics::vec3,                 py::arg("dq"), "Maps the primary part of `dq` into a 3-dimensional vector.");
    m.def("vec4"                ,&DQ_robotics::vec4,                 py::arg("dq"), "Maps the primary part of `dq` into a 4-dimensional vector.");
    m.def("vec6"                ,&DQ_robotics::vec6,                 py::arg("dq"), "Maps `dq` into a 6-dimensional vector, discarding the real part of both the primary and dual components.");
    m.def("vec8"                ,&DQ_robotics::vec8,                 py::arg("dq"), "Maps `dq` into an 8-dimensional vector.");
    m.def("normalize"           ,&DQ_robotics::normalize,            py::arg("dq"), "Returns `dq` normalized to unit norm.");
    m.def("generalized_jacobian",&DQ_robotics::generalized_jacobian, py::arg("dq"), "Returns the generalized Jacobian used in the mapping between the time derivative of the unit dual quaternion `dq` and the twist it represents.");
    m.def("sharp"               ,&DQ_robotics::sharp,                py::arg("dq"), "Returns the sharp conjugate of `dq`.");
    m.def("crossmatrix4"        ,&DQ_robotics::crossmatrix4,         py::arg("dq"), "Maps the pure quaternion `dq` into an expanded skew-symmetric matrix representation of the cross product.");
    m.def("Ad"                  ,&DQ_robotics::Ad,                   py::arg("dq1"), py::arg("dq2"), "Returns the adjoint transformation `dq1 * dq2 * dq1'`.");
    m.def("Adsharp"             ,&DQ_robotics::Adsharp,              py::arg("dq1"), py::arg("dq2"), "Returns the sharp adjoint transformation `sharp(dq1) * dq2 * dq1'`.");
    m.def("cross"               ,&DQ_robotics::cross,                py::arg("dq1"), py::arg("dq2"), "Returns the cross product between the pure dual quaternions `dq1` and `dq2`.");
    m.def("dot"                 ,&DQ_robotics::dot,                  py::arg("dq1"), py::arg("dq2"), "Returns the dot product between the pure dual quaternions `dq1` and `dq2`.");
    m.def("Q4"                  ,&DQ_robotics::Q4,                   py::arg("dq"), "Given the unit quaternion `dq`, returns the partial derivative of that quaternion with respect to its logarithm.");
    m.def("Q8"                  ,&DQ_robotics::Q8,                   py::arg("dq"), "Given the unit dual quaternion `dq`, returns the partial derivative of `dq` with respect to its logarithm.");

    m.def("is_unit"             ,&DQ_robotics::is_unit,              py::arg("dq"), "Returns true if `dq` is a unit norm dual quaternion, false otherwise.");
    m.def("is_pure"             ,&DQ_robotics::is_pure,              py::arg("dq"), "Returns true if `dq` is pure (i.e., `Re(dq) = 0`), false otherwise.");
    m.def("is_real"             ,&DQ_robotics::is_real,              py::arg("dq"), "Returns true if the imaginary part of `dq` is zero, false otherwise.");
    m.def("is_real_number"      ,&DQ_robotics::is_real_number,       py::arg("dq"), "Returns true if both the dual and imaginary parts of `dq` are zero, false otherwise.");
    m.def("is_quaternion"       ,&DQ_robotics::is_quaternion,        py::arg("dq"), "Returns true if the dual part of `dq` is zero, false otherwise.");
    m.def("is_pure_quaternion"  ,&DQ_robotics::is_pure_quaternion,   py::arg("dq"), "Returns true if `dq` is a pure quaternion (i.e., `Re(dq) = D(dq) = 0`), false otherwise.");
    m.def("is_line"             ,&DQ_robotics::is_line,              py::arg("dq"), "Returns true if `dq` is a Plucker line (i.e., `Re(dq) = 0` and `norm(dq) = 1`), false otherwise.");
    m.def("is_plane"            ,&DQ_robotics::is_plane,             py::arg("dq"), "Returns true if `dq` is a plane (i.e., it has unit norm and `Im(D(dq)) = 0`), false otherwise.");

    ///Namespace readonly
    m.attr("DQ_threshold") = DQ_threshold;
    m.attr("i_")           = DQ_robotics::i_;
    m.attr("j_")           = DQ_robotics::j_;
    m.attr("k_")           = DQ_robotics::k_;
    m.attr("E_")           = DQ_robotics::E_;
}
