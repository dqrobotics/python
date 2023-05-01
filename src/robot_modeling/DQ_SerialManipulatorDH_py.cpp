/**
(C) Copyright 2020 DQ Robotics Developers

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

#include "../dqrobotics_module.h"

void init_DQ_SerialManipulatorDH_py(py::module& m)
{
    /***************************************************
    *  DQ SerialManipulatorDH
    * **************************************************/
    py::class_<
            DQ_SerialManipulatorDH,
            std::shared_ptr<DQ_SerialManipulatorDH>,
            DQ_SerialManipulator
            > dqserialmanipulatordh_py(m, "DQ_SerialManipulatorDH");
    dqserialmanipulatordh_py.def(py::init<MatrixXd>());

    ///Methods
    //Concrete
    dqserialmanipulatordh_py.def("get_thetas",  &DQ_SerialManipulatorDH::get_thetas, "Retrieves the vector of thetas.");
    dqserialmanipulatordh_py.def("get_ds",      &DQ_SerialManipulatorDH::get_ds,     "Retrieves the vector of ds.");
    dqserialmanipulatordh_py.def("get_as",      &DQ_SerialManipulatorDH::get_as,     "Retrieves the vector of as.");
    dqserialmanipulatordh_py.def("get_alphas",  &DQ_SerialManipulatorDH::get_alphas, "Retrieves the vector of alphas.");
    dqserialmanipulatordh_py.def("get_types",   &DQ_SerialManipulatorDH::get_types,  "Retrieves the vector of types.");

    //Overrides from DQ_SerialManipulator
    dqserialmanipulatordh_py.def("raw_pose_jacobian",  (MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDH::raw_pose_jacobian, "Retrieves the raw pose Jacobian.");
    dqserialmanipulatordh_py.def("raw_fkm",            (DQ (DQ_SerialManipulatorDH::*)(const VectorXd&, const int&) const)&DQ_SerialManipulatorDH::raw_fkm,                 "Retrieves the raw FKM.");
    dqserialmanipulatordh_py.def("raw_pose_jacobian_derivative",(MatrixXd (DQ_SerialManipulatorDH::*)(const VectorXd&, const VectorXd&, const int&) const)
                                                                &DQ_SerialManipulatorDH::raw_pose_jacobian_derivative, "Retrieves the raw pose Jacobian derivative.");
}
