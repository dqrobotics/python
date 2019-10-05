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

#include "../../dqrobotics_module.h"

void init_DQ_VrepRobot_py(py::module& m)
{
    /*****************************************************
     *  VrepRobot
     * **************************************************/
    py::class_<DQ_VrepRobot> dqvreprobot_py(m,"DQ_VrepRobot");
    dqvreprobot_py.def("send_q_to_vrep", &DQ_VrepRobot::send_q_to_vrep, "Get joint values from vrep.");
    dqvreprobot_py.def("get_q_from_vrep", &DQ_VrepRobot::get_q_from_vrep, "Send joint values to vrep.");

}
