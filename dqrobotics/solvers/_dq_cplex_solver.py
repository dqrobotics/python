"""
# Copyright (c) 2019-2022 DQ Robotics Developers
#
#    This file is part of DQ Robotics.
#
#    DQ Robotics is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    DQ Robotics is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with DQ Robotics.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Contributors:
#   - Murilo M. Marinho, email: murilo@g.u-tokyo.ac.jp
#
# ################################################################
"""
from dqrobotics._dqrobotics._solvers import DQ_QuadraticProgrammingSolver
import numpy as np
import cplex

# https://github.com/dqrobotics/python/issues/24
class DQ_CPLEXSolver(DQ_QuadraticProgrammingSolver):
    def __init__(self):
        DQ_QuadraticProgrammingSolver.__init__(self)
        # Make and set a solver instance
        self.P = cplex.Cplex()
        self.P.objective.set_sense(self.P.objective.sense.minimize)
        self.P.set_problem_type(self.P.problem_type.QP)
        self.P.set_results_stream(results_file=None)

    def solve_quadratic_program(self, H, f, A, b, Aeq, beq):
        problem_size = H.shape[0]
        inequality_constraint_size = b.shape[0]
        equality_constraint_size = beq.shape[0]

        # Define the number of variables by naming them
        q_dot = ["q{}".format(i) for i in range(problem_size)]
        if self.P.variables.get_num() != problem_size:
            self.P.variables.delete()
            self.P.variables.add(names=q_dot)

        # Decide the range of variables
        for i in range(problem_size):
            self.P.variables.set_lower_bounds(i, -1 * cplex.infinity)
            self.P.variables.set_upper_bounds(i, cplex.infinity)

        # Set f'x
        f = f.tolist()
        for i in range(problem_size):
            self.P.objective.set_linear(i, f[i][0])

        # Set x^T H x
        H = H.tolist()
        for i in range(problem_size):
            for j in range(i, problem_size):
                self.P.objective.set_quadratic_coefficients(i, j, H[i][j])

        # Set linear inequality constraints and equality constraints
        # Define the number of total constraints by naming them
        c = ["c{}".format(i) for i in range(inequality_constraint_size + equality_constraint_size)]
        if self.P.linear_constraints.get_num() != inequality_constraint_size + equality_constraint_size:
            self.P.linear_constraints.delete()
            self.P.linear_constraints.add(names=c)

        # Inequalities
        A = A.tolist()
        b = b.tolist()
        for i in range(inequality_constraint_size):
            self.P.linear_constraints.set_linear_components(c[i], [q_dot, A[i]])
            self.P.linear_constraints.set_senses(c[i], 'L')
            self.P.linear_constraints.set_rhs(c[i], b[i][0])

        # Equalities
        Aeq = Aeq.tolist()
        beq = beq.tolist()
        for i in range(equality_constraint_size):
            self.P.linear_constraints.set_linear_components(c[inequality_constraint_size + i], [q_dot, Aeq[i]])
            self.P.linear_constraints.set_senses(c[inequality_constraint_size + i], 'E')
            self.P.linear_constraints.set_rhs(c[inequality_constraint_size + i], beq[i][0])

        # Solve the problem
        self.P.solve()
        delta_thetas = np.array(self.P.solution.get_values())

        return delta_thetas
