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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include "../dqrobotics_module.h"

void init_DQ_Math_py(py::module& m)
{
    /*****************************************************
     *  DQ_Math
     * **************************************************/
    //#include<dqrobotics/utils/DQ_Math.h>
    py::module math_py = m.def_submodule("DQ_Math","A submodule of utils");
    math_py.def("deg2rad", static_cast<double (*) (const double&)>(&DQ_robotics::deg2rad), "Converts from degrees to radians.");
    math_py.def("deg2rad", static_cast<VectorXd (*) (const VectorXd&)>(&DQ_robotics::deg2rad), "Converts from degrees to radians.");
    math_py.def("rad2deg", static_cast<double (*) (const double&)>(&DQ_robotics::rad2deg), "Converts from degrees to radians.");
    math_py.def("rad2deg", static_cast<VectorXd (*) (const VectorXd&)>(&DQ_robotics::rad2deg), "Converts from degrees to radians.");
}
