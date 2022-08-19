/**
(C) Copyright 2021 DQ Robotics Developers

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

void init_DQ_SerialManipulatorDenso_py(py::module& m)
{
    /***************************************************
    *  DQ SerialManipulatorDenso
    * **************************************************/
    py::class_<
            DQ_SerialManipulatorDenso,
            std::shared_ptr<DQ_SerialManipulatorDenso>,
            DQ_SerialManipulator> dqserialmanipulatordh_py(m, "DQ_SerialManipulatorDenso");
    dqserialmanipulatordh_py.def(py::init<MatrixXd>());

    ///Methods
    //Concrete
    dqserialmanipulatordh_py.def("get_as",      &DQ_SerialManipulatorDenso::get_as,     "Retrieves the vector of as.");
    dqserialmanipulatordh_py.def("get_bs",      &DQ_SerialManipulatorDenso::get_bs,     "Retrieves the vector of bs.");
    dqserialmanipulatordh_py.def("get_ds",      &DQ_SerialManipulatorDenso::get_ds,     "Retrieves the vector of ds.");

    dqserialmanipulatordh_py.def("get_alphas",  &DQ_SerialManipulatorDenso::get_alphas, "Retrieves the vector of alphas.");
    dqserialmanipulatordh_py.def("get_betas",   &DQ_SerialManipulatorDenso::get_betas,  "Retrieves the vector of betas.");
    dqserialmanipulatordh_py.def("get_thetas",  &DQ_SerialManipulatorDenso::get_gammas, "Retrieves the vector of gammas.");

    //Overrides from DQ_SerialManipulator
    dqserialmanipulatordh_py.def("raw_pose_jacobian",  (MatrixXd (DQ_SerialManipulatorDenso::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDenso::raw_pose_jacobian, "Retrieves the raw pose Jacobian.");
    dqserialmanipulatordh_py.def("raw_fkm",            (DQ (DQ_SerialManipulatorDenso::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDenso::raw_fkm,                 "Retrieves the raw FKM.");
}
