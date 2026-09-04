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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#include "../../dqrobotics_module.h"

/**
 * @brief Binds `DQ_SerialVrepRobot`, a serial-robot wrapper that exposes joint
 * names, target commands, velocities, and torques in CoppeliaSim, to the
 * Python module @p m.
 */
void init_DQ_SerialVrepRobot_py(py::module& m)
{
    /*****************************************************
     *  SerialVrepRobot
     * **************************************************/
    py::class_<
            DQ_SerialVrepRobot,
            std::shared_ptr<DQ_SerialVrepRobot>,
            DQ_VrepRobot
            > dqsv_robot(
                    m,
                    "DQ_SerialVrepRobot",
                    "Serial robot wrapper for exchanging joint names, target commands, velocities, and torques with CoppeliaSim.");


    dqsv_robot.def("get_joint_names",
                   &DQ_SerialVrepRobot::get_joint_names,
                   "Gets the joint names used in CoppeliaSim.");

    dqsv_robot.def("set_target_configuration_space_positions",
                   &DQ_SerialVrepRobot::set_target_configuration_space_positions,
                   py::arg("q"),
                   "Sets the target configuration-space positions in CoppeliaSim.");

    dqsv_robot.def("get_configuration_space_velocities",
                   &DQ_SerialVrepRobot::get_configuration_space_velocities,
                   "Gets the configuration-space velocities from CoppeliaSim.");
    dqsv_robot.def("set_target_configuration_space_velocities",
                   &DQ_SerialVrepRobot::set_target_configuration_space_velocities,
                   py::arg("q_dot"),
                   "Sets the target configuration-space velocities in CoppeliaSim.");

    dqsv_robot.def("set_configuration_space_torques",
                   &DQ_SerialVrepRobot::set_configuration_space_torques,
                   py::arg("torques"),
                   "Sets the configuration-space torques in CoppeliaSim.");
    dqsv_robot.def("get_configuration_space_torques",
                   &DQ_SerialVrepRobot::get_configuration_space_torques,
                   "Gets the configuration-space torques from CoppeliaSim.");

    //Deprecated
    dqsv_robot.def("send_q_target_to_vrep",
                   &DQ_SerialVrepRobot::send_q_target_to_vrep,
                   py::arg("q"),
                   "Deprecated alias for setting the target configuration-space positions in CoppeliaSim.");
}
