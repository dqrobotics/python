#include "dqrobotics_module.h"

//https://pybind11.readthedocs.io/en/stable/advanced/classes.html
class DQ_QuadraticProgrammingSolverPy : public DQ_QuadraticProgrammingSolver
{
public:
    //protected:
    /* Inherit the constructors */
    using DQ_QuadraticProgrammingSolver::DQ_QuadraticProgrammingSolver;
    //DQ_QuadraticProgrammingSolverPy() = default;

    /* Trampoline (need one for each virtual function) */
    VectorXd solve_quadratic_program(const MatrixXd &H, const MatrixXd &f, const MatrixXd A, const MatrixXd &b, const MatrixXd &Aeq, const MatrixXd &beq) override{
        PYBIND11_OVERLOAD_PURE(
                    VectorXd,                       /* Return type */
                    DQ_QuadraticProgrammingSolver,  /* Parent class */
                    solve_quadratic_program,        /* Name of function in C++ (must match Python name) */
                    H, f, A, b, Aeq, beq            /* Argument(s) */
                    )
    }
};



PYBIND11_MODULE(dqrobotics, m) {

    //DQ Class
    init_DQ_py(m);

    /*****************************************************
     *  Utils
     * **************************************************/
    //dqrobotics/utils/
    py::module utils_py = m.def_submodule("utils","A submodule of dqrobotics");

    //DQ_LinearAlgebra
    init_DQ_LinearAlgebra_py(utils_py);

    //DQ_Geometry
    init_DQ_Geometry_py(utils_py);

    /*****************************************************
     *  Robots Kinematic Models
     * **************************************************/
    py::module robots_py = m.def_submodule("robots", "A submodule of dqrobotics");

    //#include <dqrobotics/robots/Ax18ManipulatorRobot.h>
    py::class_<Ax18ManipulatorRobot> ax18manipulatorrobot_py(robots_py, "Ax18ManipulatorRobot");
    ax18manipulatorrobot_py.def_static("kinematics",&Ax18ManipulatorRobot::kinematics,"Returns the kinematics of the Ax18ManipulatorRobot");

    //#include <dqrobotics/robots/BarrettWamArmRobot.h>
    py::class_<BarrettWamArmRobot> barrettwamarmrobot_py(robots_py, "BarrettWamArmRobot");
    barrettwamarmrobot_py.def_static("kinematics",&BarrettWamArmRobot::kinematics,"Returns the kinematics of the BarrettWamArmRobot");

    //#include <dqrobotics/robots/ComauSmartSixRobot.h>
    py::class_<ComauSmartSixRobot> comausmartsixrobot_py(robots_py, "ComauSmartSixRobot");
    comausmartsixrobot_py.def_static("kinematics",&ComauSmartSixRobot::kinematics,"Returns the kinematics of the ComauSmartSixRobot");

    //#include <dqrobotics/robots/KukaLw4Robot.h>
    py::class_<KukaLw4Robot> kukalw4robot_py(robots_py, "KukaLw4Robot");
    kukalw4robot_py.def_static("kinematics",&KukaLw4Robot::kinematics,"Returns the kinematics of the KukaLw4Robot");

    /*****************************************************
     *  Robot Modeling <dqrobotics/robot_modeling/...>
     * **************************************************/
    py::module robot_modeling = m.def_submodule("robot_modeling", "The robot_modeling submodule of dqrobotics");

    //DQ_Kinematics
    init_DQ_Kinematics_py(robot_modeling);

    //DQ_SerialManipulator
    init_DQ_SerialManipulator_py(robot_modeling);

    //DQ_CooperativeDualTaskSpace
    init_DQ_CooperativeDualTaskSpace_py(robot_modeling);

    //DQ_MobileBase
    init_DQ_MobileBase_py(robot_modeling);

    //DQ_HolonomicBase
    init_DQ_HolonomicBase_py(robot_modeling);

    //DQ_DifferentialDriveRobot
    init_DQ_DifferentialDriveRobot_py(robot_modeling);

    //DQ_WholeBody
    init_DQ_WholeBody_py(robot_modeling);

    /*****************************************************
     *  Solvers <dqrobotics/solvers/...>
     * **************************************************/
    py::module solvers = m.def_submodule("solvers", "The solvers submodule of dqrobotics");

    /*****************************************************
     *  DQ DQ_QuadraticProgrammingSolver
     * **************************************************/
    py::class_<DQ_QuadraticProgrammingSolver, DQ_QuadraticProgrammingSolverPy> dqquadraticprogrammingsolver_py(solvers,"DQ_QuadraticProgrammingSolver");
    dqquadraticprogrammingsolver_py.def(py::init<>());
    dqquadraticprogrammingsolver_py.def("solve_quadratic_program", &DQ_QuadraticProgrammingSolver::solve_quadratic_program, "Solves a quadratic program");

    /*****************************************************
     *  Robot Control <dqrobotics/robot_control/...>
     * **************************************************/
    py::module robot_control = m.def_submodule("robot_control", "The robot_control submodule of dqrobotics");

    py::enum_<ControlObjective>(robot_control, "ControlObjective")
            .value("Line",           ControlObjective::Line)
            .value("None",           ControlObjective::None)
            .value("Pose",           ControlObjective::Pose)
            .value("Plane",          ControlObjective::Plane)
            .value("Distance",       ControlObjective::Distance)
            .value("Rotation",       ControlObjective::Rotation)
            .value("Translation",    ControlObjective::Translation)
            .export_values();

    //DQ_KinematicController
    init_DQ_KinematicController_py(robot_control);

    //DQ_TaskSpacePseudoInverseController
    init_DQ_TaskSpacePseudoInverseController_py(robot_control);

    //DQ_KinematicConstrainedController
    init_DQ_KinematicConstrainedController_py(robot_control);

    //DQ_TaskspaceQuadraticProgrammingController
    init_DQ_TaskspaceQuadraticProgrammingController_py(robot_control);

    //DQ_ClassicQPController
    init_DQ_ClassicQPController_py(robot_control);

    /*****************************************************
     *  Interfaces Submodule
     * **************************************************/
    py::module interfaces_py = m.def_submodule("interfaces", "A submodule of dqrobotics");

    /*****************************************************
     *  Vrep Submodule
     * **************************************************/
    py::module vrep_py = interfaces_py.def_submodule("vrep", "A submodule of dqrobotics");
    vrep_py.attr("VREP_OBJECTNAME_ABSOLUTE") = VREP_OBJECTNAME_ABSOLUTE;

    //DQ_VrepInterface
    init_DQ_VrepInterface_py(vrep_py);

    //DQ_VrepRobot
    init_DQ_VrepRobot_py(vrep_py);

    /*****************************************************
     *  Vrep Robots Submodule
     * **************************************************/
    py::module vreprobots_py = vrep_py.def_submodule("robots", "A submodule of dqrobotics");

    /*****************************************************
     *  LBR4pVrepRobot
     * **************************************************/
    py::class_<LBR4pVrepRobot,DQ_VrepRobot> lbr4pvreprobot_py(vreprobots_py,"LBR4pVrepRobot");
    lbr4pvreprobot_py.def(py::init<const std::string&, DQ_VrepInterface*>());

    lbr4pvreprobot_py.def("send_q_to_vrep", &LBR4pVrepRobot::send_q_to_vrep, "Send joint values to vrep.");
    lbr4pvreprobot_py.def("get_q_from_vrep", &LBR4pVrepRobot::get_q_from_vrep, "Get joint values from vrep.");
    lbr4pvreprobot_py.def("kinematics", &LBR4pVrepRobot::kinematics, "Get kinematics model.");

    /*****************************************************
     *  YouBotVrepRobot
     * **************************************************/
    py::class_<YouBotVrepRobot,DQ_VrepRobot> youbotvreprobot_py(vreprobots_py,"YouBotVrepRobot");
    youbotvreprobot_py.def(py::init<const std::string&, DQ_VrepInterface*>());

    youbotvreprobot_py.def("send_q_to_vrep", &YouBotVrepRobot::send_q_to_vrep, "Send joint values to vrep.");
    youbotvreprobot_py.def("get_q_from_vrep", &YouBotVrepRobot::get_q_from_vrep, "Get joint values from vrep.");
    youbotvreprobot_py.def("kinematics", &YouBotVrepRobot::kinematics, "Get kinematics model.");

}

