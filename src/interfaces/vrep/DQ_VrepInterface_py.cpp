/**
(C) Copyright 2019-2023 DQ Robotics Developers

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

2.  Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
        -Added the method wait_for_simulation_step_to_end()
*/

#include "../../dqrobotics_module.h"

//Default arguments added with:
//https://pybind11.readthedocs.io/en/stable/basics.html#default-args

void init_DQ_VrepInterface_py(py::module& m)
{
    /*****************************************************
     *  VrepInterface
     * **************************************************/
    py::class_<
            DQ_VrepInterface,
            std::shared_ptr<DQ_VrepInterface>
            > dqvrepinterface_py(m,"DQ_VrepInterface");
    dqvrepinterface_py.def(py::init<>());
    dqvrepinterface_py.def(py::init<std::atomic_bool*>());

    py::enum_<DQ_VrepInterface::OP_MODES>(dqvrepinterface_py, "OP_MODES")
            .value("OP_BUFFER",    DQ_VrepInterface::OP_MODES::OP_BUFFER)
            .value("OP_ONESHOT",   DQ_VrepInterface::OP_MODES::OP_ONESHOT)
            .value("OP_BLOCKING",  DQ_VrepInterface::OP_MODES::OP_BLOCKING)
            .value("OP_STREAMING", DQ_VrepInterface::OP_MODES::OP_STREAMING)
            .value("OP_AUTOMATIC", DQ_VrepInterface::OP_MODES::OP_AUTOMATIC)
            .export_values();

    py::enum_<DQ_VrepInterface::SCRIPT_TYPES>(dqvrepinterface_py, "SCRIPT_TYPES")
            .value("ST_CHILD",          DQ_VrepInterface::SCRIPT_TYPES::ST_CHILD)
            .value("ST_MAIN",           DQ_VrepInterface::SCRIPT_TYPES::ST_MAIN)
            .value("ST_CUSTOMIZATION",  DQ_VrepInterface::SCRIPT_TYPES::ST_CUSTOMIZATION)
            .export_values();

    py::enum_<DQ_VrepInterface::REFERENCE_FRAMES>(dqvrepinterface_py, "REFERENCE_FRAMES")
            .value("BODY_FRAME",          DQ_VrepInterface::REFERENCE_FRAMES::BODY_FRAME)
            .value("ABSOLUTE_FRAME",      DQ_VrepInterface::REFERENCE_FRAMES::ABSOLUTE_FRAME)
            .export_values();

    dqvrepinterface_py.def("connect",(bool (DQ_VrepInterface::*) (const int&, const int&, const int&))&DQ_VrepInterface::connect,"Connects to V-REP in local machine.");
    dqvrepinterface_py.def("connect",(bool (DQ_VrepInterface::*) (const std::string&, const int&, const int&, const int&))&DQ_VrepInterface::connect,"Connects to V-REP with a given ip.");

    dqvrepinterface_py.def("disconnect",    &DQ_VrepInterface::disconnect,"Disconnects from V-REP.");
    dqvrepinterface_py.def("disconnect_all",&DQ_VrepInterface::disconnect_all,"Disconnect all from V-REP");

    dqvrepinterface_py.def("start_simulation",&DQ_VrepInterface::start_simulation,"Start simulation");
    dqvrepinterface_py.def("stop_simulation", &DQ_VrepInterface::stop_simulation,"Stops simulation");

    dqvrepinterface_py.def("is_simulation_running",&DQ_VrepInterface::is_simulation_running,"Checks whether the simulation is running or not");

    //    void set_synchronous(const bool& flag);
    dqvrepinterface_py.def("set_synchronous", (void (DQ_VrepInterface::*) (const bool&))&DQ_VrepInterface::set_synchronous, "Sets synchronous mode");

    //void trigger_next_simulation_step();
    dqvrepinterface_py.def("trigger_next_simulation_step", &DQ_VrepInterface::trigger_next_simulation_step, "Sends a synchronization trigger signal to the server.");

    //void wait_for_simulation_step_to_end(); 
    dqvrepinterface_py.def("wait_for_simulation_step_to_end", &DQ_VrepInterface::wait_for_simulation_step_to_end, "Waits until the simulation step is finished.");

    //dqvrepinterface_py.def("get_object_handle", &DQ_VrepInterface::get_object_handle,"Gets an object handle");
    //dqvrepinterface_py.def("get_object_handles",&DQ_VrepInterface::get_object_handles,"Get object handles");

    //dqvrepinterface_py.def("get_object_translation",(DQ (DQ_VrepInterface::*) (const int&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_translation,"Gets object translation.");
    //dqvrepinterface_py.def("get_object_translation",(DQ (DQ_VrepInterface::*) (const std::string&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_translation,"Gets object translation.");
    //dqvrepinterface_py.def("get_object_translation",(DQ (DQ_VrepInterface::*) (const int&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_translation,"Gets object translation.");
    dqvrepinterface_py.def("get_object_translation",
                           (DQ (DQ_VrepInterface::*) (const std::string&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_translation,
                           "Gets object translation.",
                           py::arg("objectname")=std::string(""),
                           py::arg("relative_to_objectname")=VREP_OBJECTNAME_ABSOLUTE,
                           py::arg("opmode")=DQ_VrepInterface::OP_AUTOMATIC);

    //dqvrepinterface_py.def("set_object_translation",(void (DQ_VrepInterface::*) (const int&, const int&, const DQ&, const DQ_VrepInterface::OP_MODES&) const)&DQ_VrepInterface::set_object_translation,"Sets object translation.");
    //dqvrepinterface_py.def("set_object_translation",(void (DQ_VrepInterface::*) (const std::string&, const int&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_translation,"Sets object translation.");
    //dqvrepinterface_py.def("set_object_translation",(void (DQ_VrepInterface::*) (const int&, const std::string&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_translation,"Sets object translation.");
    dqvrepinterface_py.def("set_object_translation",
                           (void (DQ_VrepInterface::*) (const std::string&, const DQ&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_translation,
                           "Sets object translation.",
                           py::arg("objectname")=std::string(""),
                           py::arg("t")=DQ(0),
                           py::arg("relative_to_objectname")=VREP_OBJECTNAME_ABSOLUTE,
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //dqvrepinterface_py.def("get_object_rotation",(DQ (DQ_VrepInterface::*) (const int&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_rotation,"Gets object rotation.");
    //dqvrepinterface_py.def("get_object_rotation",(DQ (DQ_VrepInterface::*) (const std::string&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_rotation,"Gets object rotation.");
    //dqvrepinterface_py.def("get_object_rotation",(DQ (DQ_VrepInterface::*) (const int&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_rotation,"Gets object rotation.");
    dqvrepinterface_py.def("get_object_rotation",
                           (DQ (DQ_VrepInterface::*) (const std::string&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_rotation,
                           "Gets object rotation.",
                           py::arg("objectname")=std::string(""),
                           py::arg("relative_to_objectname")=VREP_OBJECTNAME_ABSOLUTE,
                           py::arg("opmode")=DQ_VrepInterface::OP_AUTOMATIC);

    //dqvrepinterface_py.def("set_object_rotation",(void (DQ_VrepInterface::*) (const int&, const int&, const DQ&, const DQ_VrepInterface::OP_MODES&) const)&DQ_VrepInterface::set_object_rotation,"Sets object rotation.");
    //dqvrepinterface_py.def("set_object_rotation",(void (DQ_VrepInterface::*) (const std::string&, const int&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_rotation,"Sets object rotation.");
    //dqvrepinterface_py.def("set_object_rotation",(void (DQ_VrepInterface::*) (const int&, const std::string&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_rotation,"Sets object rotation.");
    dqvrepinterface_py.def("set_object_rotation",
                           (void (DQ_VrepInterface::*) (const std::string&, const DQ&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_rotation,
                           "Sets object rotation.",
                           py::arg("objectname")=std::string(""),
                           py::arg("r")=DQ(1),
                           py::arg("relative_to_objectname")=VREP_OBJECTNAME_ABSOLUTE,
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //dqvrepinterface_py.def("get_object_pose",(DQ (DQ_VrepInterface::*) (const int&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_pose,"Gets object pose.");
    //dqvrepinterface_py.def("get_object_pose",(DQ (DQ_VrepInterface::*) (const std::string&, const int&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_pose,"Gets object pose.");
    //dqvrepinterface_py.def("get_object_pose",(DQ (DQ_VrepInterface::*) (const int&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_pose,"Gets object pose.");
    dqvrepinterface_py.def("get_object_pose",
                           (DQ (DQ_VrepInterface::*) (const std::string&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_object_pose,
                           "Gets object pose.",
                           py::arg("objectname")=std::string(""),
                           py::arg("relative_to_objectname")=VREP_OBJECTNAME_ABSOLUTE,
                           py::arg("opmode")=DQ_VrepInterface::OP_AUTOMATIC);

    //dqvrepinterface_py.def("set_object_pose",(void (DQ_VrepInterface::*) (const int&, const int&, const DQ&, const DQ_VrepInterface::OP_MODES&) const)&DQ_VrepInterface::set_object_pose,"Sets object pose.");
    //dqvrepinterface_py.def("set_object_pose",(void (DQ_VrepInterface::*) (const std::string&, const int&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_pose,"Sets object pose.");
    //dqvrepinterface_py.def("set_object_pose",(void (DQ_VrepInterface::*) (const int&, const std::string&, const DQ&,  const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_pose,"Sets object pose.");
    dqvrepinterface_py.def("set_object_pose",
                           (void (DQ_VrepInterface::*) (const std::string&, const DQ&, const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_object_pose,
                           "Sets object pose.",
                           py::arg("objectname")=std::string(""),
                           py::arg("h")=DQ(1),
                           py::arg("relative_to_objectname")=VREP_OBJECTNAME_ABSOLUTE,
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    dqvrepinterface_py.def("get_object_poses",&DQ_VrepInterface::get_object_poses,"Get the poses of many objects");
    dqvrepinterface_py.def("set_object_poses",&DQ_VrepInterface::set_object_poses,"Set object poses of many objects");

    //dqvrepinterface_py.def("set_joint_position",(void (DQ_VrepInterface::*) (const int&, const double&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::set_joint_position,"Set joint position");
    dqvrepinterface_py.def("set_joint_position",
                           (void (DQ_VrepInterface::*) (const std::string&, const double&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_position,
                           "Set joint position",
                           py::arg("jointname")=std::string(""),
                           py::arg("angle_rad")=0.0,
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //dqvrepinterface_py.def("set_joint_target_position",(void (DQ_VrepInterface::*) (const int&, const double&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::set_joint_target_position,"Set joint position");
    dqvrepinterface_py.def("set_joint_target_position",
                           (void (DQ_VrepInterface::*) (const std::string&, const double&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_target_position,
                           "Set joint position",
                           py::arg("jointname")=std::string(""),
                           py::arg("angle_rad")=0.0,
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //dqvrepinterface_py.def("get_joint_position",(double (DQ_VrepInterface::*) (const int&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::get_joint_position,"Get joint position");
    dqvrepinterface_py.def("get_joint_position",
                           (double (DQ_VrepInterface::*) (const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_joint_position,
                           "Get joint position",
                           py::arg("jointname")=std::string(""),
                           py::arg("opmode")=DQ_VrepInterface::OP_AUTOMATIC);

    //dqvrepinterface_py.def("set_joint_positions",(void (DQ_VrepInterface::*) (const std::vector<int>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::set_joint_positions,"Set joint positions");
    dqvrepinterface_py.def("set_joint_positions",
                           (void (DQ_VrepInterface::*) (const std::vector<std::string>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_positions,
                           "Set joint positions",
                           py::arg("jointnames")=std::vector<std::string>(),
                           py::arg("angles_rad")=VectorXd::Zero(1),
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //dqvrepinterface_py.def("set_joint_target_positions",(void (DQ_VrepInterface::*) (const std::vector<int>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::set_joint_target_positions,"Set joint positions");
    dqvrepinterface_py.def("set_joint_target_positions",
                           (void (DQ_VrepInterface::*) (const std::vector<std::string>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_target_positions,
                           "Set joint positions",
                           py::arg("jointnames")=std::vector<std::string>(),
                           py::arg("angles_rad")=VectorXd::Zero(1),
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //dqvrepinterface_py.def("get_joint_positions",(VectorXd (DQ_VrepInterface::*) (const std::vector<int>&, const DQ_VrepInterface::OP_MODES&) const)  &DQ_VrepInterface::get_joint_positions,"Get joint positions");
    dqvrepinterface_py.def("get_joint_positions",
                           (VectorXd (DQ_VrepInterface::*) (const std::vector<std::string>&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_joint_positions,
                           "Get joint positions",
                           py::arg("jointnames")=std::vector<std::string>(),
                           py::arg("opmode")=DQ_VrepInterface::OP_AUTOMATIC);


    //void set_joint_target_velocity(const std::string& jointname, const double& angle_dot_rad, const OP_MODES& opmode=OP_ONESHOT);
    dqvrepinterface_py.def("set_joint_target_velocity",
                           (void (DQ_VrepInterface::*) (const std::string&, const double&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_target_velocity,
                           "Set joint velocity",
                           py::arg("jointname")=std::string(""),
                           py::arg("angle_dot_rad")=0.0,
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //void set_joint_target_velocities(const std::vector<std::string>& jointnames, const VectorXd& angles_dot_rad, const OP_MODES& opmode=OP_ONESHOT);
    dqvrepinterface_py.def("set_joint_target_velocities",
                           (void (DQ_VrepInterface::*) (const std::vector<std::string>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_target_velocities,
                           "Set joint velcoties",
                           py::arg("jointnames")=std::vector<std::string>(),
                           py::arg("angles_dot_rad")=VectorXd::Zero(1),
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //double   get_joint_velocity(const std::string& jointname, const OP_MODES& opmode=OP_AUTOMATIC);
    dqvrepinterface_py.def("get_joint_velocity",
                           (double (DQ_VrepInterface::*) (const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_joint_velocity,
                           "Get joint velocity",
                           py::arg("jointname")=std::string(""),
                           py::arg("opmode")=DQ_VrepInterface::OP_AUTOMATIC);

    //VectorXd get_joint_velocities(const std::vector<std::string>& jointnames, const OP_MODES& opmode=OP_AUTOMATIC);
    dqvrepinterface_py.def("get_joint_velocities",
                           (VectorXd (DQ_VrepInterface::*) (const std::vector<std::string>&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_joint_velocities,
                           "Get joint velocities",
                           py::arg("jointnames")=std::vector<std::string>(),
                           py::arg("opmode")=DQ_VrepInterface::OP_AUTOMATIC);


    //void     set_joint_torque(const std::string& jointname, const double& torque, const OP_MODES& opmode=OP_ONESHOT);
    dqvrepinterface_py.def("set_joint_torque",
                           (void (DQ_VrepInterface::*) (const std::string&, const double&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_torque,
                           "Set joint torque",
                           py::arg("jointname")=std::string(""),
                           py::arg("torque")=0.0,
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //void     set_joint_torques(const std::vector<std::string>& jointnames, const VectorXd& torques, const OP_MODES& opmode=OP_ONESHOT);
    dqvrepinterface_py.def("set_joint_torques",
                           (void (DQ_VrepInterface::*) (const std::vector<std::string>&, const VectorXd&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::set_joint_torques,
                           "Set joint torques",
                           py::arg("jointnames")=std::vector<std::string>(),
                           py::arg("torques")=VectorXd::Zero(1),
                           py::arg("opmode")=DQ_VrepInterface::OP_ONESHOT);

    //double   get_joint_torque(const std::string& jointname, const OP_MODES& opmode=OP_AUTOMATIC);
    dqvrepinterface_py.def("get_joint_torque",
                           (double (DQ_VrepInterface::*) (const std::string&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_joint_torque,
                           "Get joint torque",
                           py::arg("jointname")=std::string(""),
                           py::arg("opmode")=DQ_VrepInterface::OP_AUTOMATIC);

    //VectorXd get_joint_torques(const std::vector<std::string>& jointnames, const OP_MODES& opmode=OP_AUTOMATIC);
    dqvrepinterface_py.def("get_joint_torques",
                           (VectorXd (DQ_VrepInterface::*) (const std::vector<std::string>&, const DQ_VrepInterface::OP_MODES&))&DQ_VrepInterface::get_joint_torques,
                           "Get joint torques",
                           py::arg("jointnames")=std::vector<std::string>(),
                           py::arg("opmode")=DQ_VrepInterface::OP_AUTOMATIC);

    //double get_mass(const std::string& link_name, const std::string& function_name = "get_mass", const std::string& obj_name= "DQRoboticsApiCommandServer");
    dqvrepinterface_py.def("get_mass",
                           (double (DQ_VrepInterface::*) (const std::string&, const std::string&, const std::string&))&DQ_VrepInterface::get_mass,
                           "Get the mass of an object from CoppeliaSim",
                           py::arg("linkname")=std::string(""),
                           py::arg("function_name")=std::string("get_mass"),
                           py::arg("obj_name")=std::string("DQRoboticsApiCommandServer"));

    //DQ get_center_of_mass(const std::string& link_name,
    //                            const REFERENCE_FRAMES& reference_frame=BODY_FRAME,
    //                            const std::string& function_name = "get_center_of_mass",
    //                            const std::string& obj_name= "DQRoboticsApiCommandServer");
    dqvrepinterface_py.def("get_center_of_mass",
                           (DQ (DQ_VrepInterface::*) (const std::string&, const DQ_VrepInterface::REFERENCE_FRAMES&,
                                                            const std::string&, const std::string&))&DQ_VrepInterface::get_center_of_mass,
                           "Get the center of mass of an object from CoppeliaSim",
                           py::arg("linkname")=std::string(""),
                           py::arg("reference_frame")=DQ_VrepInterface::BODY_FRAME,
                           py::arg("function_name")=std::string("get_center_of_mass"),
                           py::arg("obj_name")=std::string("DQRoboticsApiCommandServer"));

    //MatrixXd get_inertia_matrix(const std::string& link_name,
    //                            const REFERENCE_FRAMES& reference_frame=BODY_FRAME,
    //                            const std::string& function_name = "get_inertia",
    //                            const std::string& obj_name= "DQRoboticsApiCommandServer");
    dqvrepinterface_py.def("get_inertia_matrix",
                           (MatrixXd (DQ_VrepInterface::*) (const std::string&, const DQ_VrepInterface::REFERENCE_FRAMES&,
                                                            const std::string&, const std::string&))&DQ_VrepInterface::get_inertia_matrix,
                           "Get the inertia matrix of an object from CoppeliaSim",
                           py::arg("linkname")=std::string(""),
                           py::arg("reference_frame")=DQ_VrepInterface::BODY_FRAME,
                           py::arg("function_name")=std::string("get_inertia"),
                           py::arg("obj_name")=std::string("DQRoboticsApiCommandServer"));

}
