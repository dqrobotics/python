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

#include "dqrobotics_module.h"

void init_DQ_py(py::module& m)
{
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
}
