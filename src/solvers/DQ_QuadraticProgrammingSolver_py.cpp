#include "../dqrobotics_module.h"

//https://pybind11.readthedocs.io/en/stable/advanced/classes.html
//Trampoline class
class DQ_QuadraticProgrammingSolverPy : public DQ_QuadraticProgrammingSolver
{
protected:
    /* Inherit the constructors */
    using DQ_QuadraticProgrammingSolver::DQ_QuadraticProgrammingSolver;
public:
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

void init_DQ_QuadraticProgrammingSolver_py(py::module& m)
{
    /*****************************************************
     *  DQ_QuadraticProgrammingSolver
     * **************************************************/
    py::class_<DQ_QuadraticProgrammingSolver, DQ_QuadraticProgrammingSolverPy> dqquadraticprogrammingsolver_py(m,"DQ_QuadraticProgrammingSolver");
    dqquadraticprogrammingsolver_py.def(py::init<>());
    dqquadraticprogrammingsolver_py.def("solve_quadratic_program", &DQ_QuadraticProgrammingSolver::solve_quadratic_program, "Solves a quadratic program");
}
