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
import quadprog

class DQ_QuadprogSolver(DQ_QuadraticProgrammingSolver):
    def __init__(self):
        DQ_QuadraticProgrammingSolver.__init__(self)
        self.equality_constraints_tolerance = 0  # default of np.finfo(np.float64).eps is already included in the solver
        pass

    def set_equality_constraints_tolerance(self, tolerance):
        """
        Set allowed tolerance for the equality constraints
        :param tolerance: Tolerance allowed for equality constraints
        """
        self.equality_constraints_tolerance = tolerance

    def get_equality_constraints_tolerance(self):
        """
        Get allowed tolerance for the equality constraints
        :return: Current tolerance
        """
        return self.equality_constraints_tolerance

    def solve_quadratic_program(self, H, f, A, b, Aeq, beq):
        """
         Solves the following quadratic program
            min(x)  0.5*x'Hx + f'x
            s.t.    Ax <= b
                    Aeqx = beq.
         Method signature is compatible with MATLAB's 'quadprog'.
         :param H: the n x n matrix of the quadratic coeficitients of the decision variables.
         :param f: the n x 1 vector of the linear coeficients of the decision variables.
         :param A: the m x n matrix of inequality constraints.
         :param b: the m x 1 value for the inequality constraints.
         :param Aeq: the m x n matrix of equality constraints.
         :param beq: the m x 1 value for the inequality constraints.
         :return: the optimal x
        """
        A_internal = A
        b_internal = b
        if Aeq is not None and beq is not None:
            if Aeq.shape == (0, 0) or beq.shape == 0:
                pass
            else:
                A_internal = np.vstack([A, Aeq, -Aeq])
                beq = beq.reshape(-1)
                b_internal = np.concatenate([b.reshape(-1), beq + self.equality_constraints_tolerance,
                                             -beq + self.equality_constraints_tolerance])
        if A_internal.shape == (0, 0) or b_internal.shape == 0:
            # Calls from DQRobotics CPP will trigger this condition
            A_internal = np.zeros((1, H.shape[0]))
            b_internal = np.zeros(1)

        (x, f, xu, iterations, lagrangian, iact) = quadprog.solve_qp(G=H,
                                                                     a=-f,
                                                                     C=-np.transpose(A_internal),
                                                                     b=-b_internal)
        return x
