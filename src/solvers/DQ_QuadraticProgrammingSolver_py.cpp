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
    VectorXd solve_quadratic_program(const MatrixXd &H, const VectorXd &f, const MatrixXd A, const VectorXd &b, const MatrixXd &Aeq, const VectorXd &beq) override{
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
