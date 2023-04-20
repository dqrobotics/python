/**
(C) Copyright 2023 DQ Robotics Developers

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

void init_DQ_SerialVrepRobot_py(py::module& m)
{
    /*****************************************************
     *  SerialVrepRobot
     * **************************************************/
    py::class_<
            DQ_SerialVrepRobot,
            std::shared_ptr<DQ_SerialVrepRobot>,
            DQ_VrepRobot
            > dqsv_robot(m,"DQ_SerialVrepRobot");

    dqsv_robot.def("send_q_target_to_vrep", &DQ_SerialVrepRobot::send_q_target_to_vrep, "Send target joint values to CoppeliaSim.");
}
