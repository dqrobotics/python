from dqrobotics._dqrobotics._solvers import *

try:
    import quadprog
    import numpy as np


    class DQ_QuadprogSolver(DQ_QuadraticProgrammingSolver):
        def __init__(self):
            DQ_QuadraticProgrammingSolver.__init__(self)
            self.equality_constraints_tolerance = 0 # default of np.finfo(np.float64).eps is already included in the solver
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
            if Aeq is not None and beq is not None:
                A = np.vstack([A, Aeq, -Aeq])
                beq = beq.reshape(-1)
                b = np.concatenate([b.reshape(-1), beq+self.equality_constraints_tolerance, -beq+self.equality_constraints_tolerance])

            (x, f, xu, iterations, lagrangian, iact) = quadprog.solve_qp(G= H,
                                                                         a=-f,
                                                                         C=-np.transpose(A),
                                                                         b=-b)
            return x
except:
    pass

try:
    import cplex
    import numpy as np


    # https://github.com/dqrobotics/python/issues/24
    class DQ_CPLEXSolver():
        def __init__(self):
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

except:
    pass
