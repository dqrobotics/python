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

2.  Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
        - Added bindings for the following methods
          set_stepping_mode(), get_object_handle(), get_object_handle()
          get_joint_{velocities, torques}(), set_joint_target_velocities(),set_joint_torques()

*/

#include "../../dqrobotics_module.h"

//Default arguments added with:
//https://pybind11.readthedocs.io/en/stable/basics.html#default-args

void init_DQ_CoppeliaSimInterfaceZMQ_py(py::module& m)
{
    /*****************************************************
     *  VrepInterface
     * **************************************************/
    py::class_<
            DQ_CoppeliaSimInterfaceZMQ,
            std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>
            > dqcsinterfacezmq_py(m,"DQ_CoppeliaSimInterfaceZMQ");
    dqcsinterfacezmq_py.def(py::init<>());

    dqcsinterfacezmq_py.def("connect",(bool (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const int&, const int&))&DQ_CoppeliaSimInterfaceZMQ::connect,
                            py::arg("host") = "localhost", py::arg("port") = 23000, py::arg("TIMEOUT_IN_MILISECONDS") = 2000,
                            "establishes a connection between the client (your code) and the host (the computer running the CoppeliaSim scene.");
    dqcsinterfacezmq_py.def("connect",(bool (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const int&, const int&, const int&))&DQ_CoppeliaSimInterfaceZMQ::connect,"Connects to CoppeliaSim with a given ip.");

    dqcsinterfacezmq_py.def("disconnect",    &DQ_CoppeliaSimInterfaceZMQ::disconnect,"Disconnects from CoppeliaSim.");
    dqcsinterfacezmq_py.def("disconnect_all",&DQ_CoppeliaSimInterfaceZMQ::disconnect_all,"Disconnect all from CoppeliaSim");

    dqcsinterfacezmq_py.def("start_simulation",&DQ_CoppeliaSimInterfaceZMQ::start_simulation,"Start simulation");
    dqcsinterfacezmq_py.def("stop_simulation", &DQ_CoppeliaSimInterfaceZMQ::stop_simulation,"Stops simulation");

    dqcsinterfacezmq_py.def("set_stepping_mode", (void (DQ_CoppeliaSimInterfaceZMQ::*) (const bool&))&DQ_CoppeliaSimInterfaceZMQ::set_stepping_mode, "enables or disables the stepping mode (formerly known as synchronous mode).");
    dqcsinterfacezmq_py.def("set_synchronous", (void (DQ_CoppeliaSimInterfaceZMQ::*) (const bool&))&DQ_CoppeliaSimInterfaceZMQ::set_synchronous, "Sets synchronous mode");

    dqcsinterfacezmq_py.def("trigger_next_simulation_step", &DQ_CoppeliaSimInterfaceZMQ::trigger_next_simulation_step, "Sends a synchronization trigger signal to the server.");

    dqcsinterfacezmq_py.def("wait_for_simulation_step_to_end", &DQ_CoppeliaSimInterfaceZMQ::wait_for_simulation_step_to_end, "Waits until the simulation step is finished.");

    dqcsinterfacezmq_py.def("get_object_translation",
                           (DQ (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&))&DQ_CoppeliaSimInterfaceZMQ::get_object_translation,
                           "Gets object translation.");

    dqcsinterfacezmq_py.def("set_object_translation",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const DQ&))&DQ_CoppeliaSimInterfaceZMQ::set_object_translation,
                           "Sets object translation.");

    dqcsinterfacezmq_py.def("get_object_rotation",
                           (DQ (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&))&DQ_CoppeliaSimInterfaceZMQ::get_object_rotation,
                           "Gets object rotation.");

    dqcsinterfacezmq_py.def("set_object_rotation",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const DQ&))&DQ_CoppeliaSimInterfaceZMQ::set_object_rotation,
                           "Sets object rotation.");

    dqcsinterfacezmq_py.def("get_object_pose",
                           (DQ (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&))&DQ_CoppeliaSimInterfaceZMQ::get_object_pose,
                           "Gets object pose.");

    dqcsinterfacezmq_py.def("set_object_pose",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&, const DQ&))&DQ_CoppeliaSimInterfaceZMQ::set_object_pose,
                           "Sets object pose.");

    dqcsinterfacezmq_py.def("get_object_handle",
                            (int (DQ_CoppeliaSimInterfaceZMQ::*) (const std::string&))&::DQ_CoppeliaSimInterfaceZMQ::get_object_handle,
                            "gets the object handle from CoppeliaSim.");

    dqcsinterfacezmq_py.def("get_object_handles",
                            (VectorXd (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&))&DQ_CoppeliaSimInterfaceZMQ::get_object_handles,
                            "returns a vector containing the object handles.");

    dqcsinterfacezmq_py.def("set_joint_positions",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&, const VectorXd&))&DQ_CoppeliaSimInterfaceZMQ::set_joint_positions,
                           "Set joint positions");

    dqcsinterfacezmq_py.def("set_joint_target_positions",
                           (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&, const VectorXd&))&DQ_CoppeliaSimInterfaceZMQ::set_joint_target_positions,
                           "Set joint positions");

    dqcsinterfacezmq_py.def("get_joint_positions",
                           (VectorXd (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&))&DQ_CoppeliaSimInterfaceZMQ::get_joint_positions,
                           "Get joint positions");

    dqcsinterfacezmq_py.def("get_joint_velocities",
                            (VectorXd (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&))&DQ_CoppeliaSimInterfaceZMQ::get_joint_velocities,
                            "gets the joint velocities in the CoppeliaSim scene.");

    dqcsinterfacezmq_py.def("set_joint_target_velocities",
                            (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&, const VectorXd&))&DQ_CoppeliaSimInterfaceZMQ::set_joint_target_velocities,
                            "sets the joint target velocities in the CoppeliaSim scene. "
                            "This method requires a dynamics enabled scene, and joints in dynamic mode with velocity control mode.");

    dqcsinterfacezmq_py.def("get_joint_torques",
                            (VectorXd (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&))&DQ_CoppeliaSimInterfaceZMQ::get_joint_torques,
                            "gets the joint torques in the CoppeliaSim scene.");

    dqcsinterfacezmq_py.def("set_joint_torques",
                            (void (DQ_CoppeliaSimInterfaceZMQ::*) (const std::vector<std::string>&, const VectorXd&))&DQ_CoppeliaSimInterfaceZMQ::set_joint_torques,
                            "sets the joint torques in the CoppeliaSim scene. "
                            "This method requires a dynamics enabled scene, and joints in dynamic mode with force control mode.");


}
