#include "../../dqrobotics_module.h"

//Default arguments added with:
//https://pybind11.readthedocs.io/en/stable/basics.html#default-args

void init_DQ_VrepInterface_py(py::module& m)
{

    /*****************************************************
     *  VrepInterface
     * **************************************************/
    py::class_<DQ_VrepInterface> dqvrepinterface_py(m,"DQ_VrepInterface");
    dqvrepinterface_py.def(py::init<>());
    dqvrepinterface_py.def(py::init<std::atomic_bool*>());

    py::enum_<DQ_VrepInterface::OP_MODES>(dqvrepinterface_py, "OP_MODES")
            .value("OP_BUFFER",    DQ_VrepInterface::OP_MODES::OP_BUFFER)
            .value("OP_ONESHOT",   DQ_VrepInterface::OP_MODES::OP_ONESHOT)
            .value("OP_BLOCKING",  DQ_VrepInterface::OP_MODES::OP_BLOCKING)
            .value("OP_STREAMING", DQ_VrepInterface::OP_MODES::OP_STREAMING)
            .value("OP_AUTOMATIC", DQ_VrepInterface::OP_MODES::OP_AUTOMATIC)
            .export_values();

    dqvrepinterface_py.def("connect",(bool (DQ_VrepInterface::*) (const int&, const int&, const int&))&DQ_VrepInterface::connect,"Connects to V-REP in local machine.");
    dqvrepinterface_py.def("connect",(bool (DQ_VrepInterface::*) (const std::string&, const int&, const int&, const int&))&DQ_VrepInterface::connect,"Connects to V-REP with a given ip.");

    dqvrepinterface_py.def("disconnect",    &DQ_VrepInterface::disconnect,"Disconnects from V-REP.");
    dqvrepinterface_py.def("disconnect_all",&DQ_VrepInterface::disconnect_all,"Disconnect all from V-REP");

    dqvrepinterface_py.def("start_simulation",&DQ_VrepInterface::start_simulation,"Start simulation");
    dqvrepinterface_py.def("stop_simulation", &DQ_VrepInterface::stop_simulation,"Stops simulation");

    dqvrepinterface_py.def("is_simulation_running",&DQ_VrepInterface::is_simulation_running,"Checks whether the simulation is running or not");

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
}
