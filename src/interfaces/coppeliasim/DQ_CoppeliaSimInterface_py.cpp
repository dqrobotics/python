/**
(C) Copyright 2019-2025 DQ Robotics Developers

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
1.  Murilo M. Marinho        (murilomarinho@ieee.org)
        - Initial implementation.
*/

#include "../../dqrobotics_module.h"

void init_DQ_CoppeliaSimInterface_py(py::module& m)
{
    /***************************************************
    *  DQ SerialManipulator
    * **************************************************/
    py::class_<
            DQ_CoppeliaSimInterface,
            std::shared_ptr<DQ_CoppeliaSimInterface>
            > c(m, "DQ_SerialManipulator");

    ///Methods
    //Virtual
    //virtual bool connect(const std::string& host, const int& port, const int&TIMEOUT_IN_MILISECONDS) = 0;
    c.def("connect", &DQ_CoppeliaSimInterface::connect,"Connects to CoppeliaSim.");
    //virtual void trigger_next_simulation_step() const = 0;
    c.def("trigger_next_simulation_step", &DQ_CoppeliaSimInterface::trigger_next_simulation_step,"Triggers the next simulation step.");
    //virtual void set_stepping_mode(const bool& flag) const = 0;
    c.def("set_stepping_mode", &DQ_CoppeliaSimInterface::set_stepping_mode,"Sets the stepping mode.");
    //virtual void start_simulation() const = 0;
    c.def("start_simulation", &DQ_CoppeliaSimInterface::start_simulation,"Starts the simulation.");
    //virtual void stop_simulation()  const = 0;
    c.def("stop_simulation", &DQ_CoppeliaSimInterface::stop_simulation,"Stops the simulation.");

    //virtual int get_object_handle(const std::string& objectname) = 0;
    c.def("get_object_handle", &DQ_CoppeliaSimInterface::get_object_handle,"Gets the object handle.");
    //virtual std::vector<int> get_object_handles(const std::vector<std::string>& objectnames) = 0;
    c.def("get_object_handles", &DQ_CoppeliaSimInterface::get_object_handles,"Gets the object handles.");

    //virtual DQ get_object_translation(const std::string& objectname) = 0;
    c.def("get_object_translation", &DQ_CoppeliaSimInterface::get_object_translation,"Gets the object translation.");
    //virtual void set_object_translation(const std::string& objectname, const DQ& t) = 0;
    c.def("set_object_translation", &DQ_CoppeliaSimInterface::set_object_translation,"Sets the object translation.");

    //virtual DQ get_object_rotation (const std::string& objectname) = 0;
    c.def("get_object_rotation", &DQ_CoppeliaSimInterface::get_object_rotation,"Gets the object rotation.");
    //virtual void set_object_rotation (const std::string& objectname, const DQ& r) = 0;
    c.def("set_object_rotation", &DQ_CoppeliaSimInterface::set_object_rotation,"Sets the object rotation.");

    //virtual DQ   get_object_pose       (const std::string& objectname) = 0;
    c.def("get_object_pose", &DQ_CoppeliaSimInterface::get_object_pose,"Gets the object pose.");
    //virtual void set_object_pose       (const std::string& objectname, const DQ& h) = 0;
    c.def("set_object_pose", &DQ_CoppeliaSimInterface::set_object_pose,"Sets the object pose.");

    //virtual VectorXd get_joint_positions(const std::vector<std::string>& jointnames) = 0;
    c.def("get_joint_positions", &DQ_CoppeliaSimInterface::get_joint_positions, "Gets the joint positions.");
    //virtual void     set_joint_positions(const std::vector<std::string>& jointnames, const VectorXd& joint_positions) = 0;
    c.def("set_joint_positions", &DQ_CoppeliaSimInterface::set_joint_positions, "Sets the joint positions.");
    //virtual void     set_joint_target_positions(const std::vector<std::string>& jointnames, const VectorXd& joint_target_positions) = 0;
    c.def("set_joint_target_positions", &DQ_CoppeliaSimInterface::set_joint_target_positions, "Sets the joint target positions.");

    //virtual VectorXd get_joint_velocities(const std::vector<std::string>& jointnames) = 0;
    c.def("get_joint_velocities", &DQ_CoppeliaSimInterface::get_joint_velocities, "Gets the joint velocities.");
    //virtual void     set_joint_target_velocities(const std::vector<std::string>& jointnames, const VectorXd& joint_target_velocities) = 0;
    c.def("set_joint_target_velocities", &DQ_CoppeliaSimInterface::set_joint_target_velocities, "Sets the target joint velocities");

    //virtual void     set_joint_torques(const std::vector<std::string>& jointnames, const VectorXd& torques) = 0;
    c.def("set_joint_torques", &DQ_CoppeliaSimInterface::set_joint_torques, "Sets the joint torques");
    //virtual VectorXd get_joint_torques(const std::vector<std::string>& jointnames) = 0;
    c.def("get_joint_torques", &DQ_CoppeliaSimInterface::get_joint_torques, "Gets the joint torques");

}
