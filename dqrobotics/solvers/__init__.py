from dqrobotics._dqrobotics._solvers import *

try:
    import quadprog
    import numpy as np
    class DQ_QuadprogSolver(DQ_QuadraticProgrammingSolver):
        def __init__(self):
            DQ_QuadraticProgrammingSolver.__init__(self)
            pass

        def solve_quadratic_program(self, H, f, A, b, Aeq, beq):
            # Ignoring Aeq and beq because quadprog doesn't handle them directly

            # Saving shape to remove the singleton dimension
            (s1, s2) = np.shape(f)
            (s3, s4) = np.shape(b)

            (x, f, xu, iterations, lagrangian, iact) = quadprog.solve_qp(G=2*H,
                                                                         a=-np.reshape(f, s1),
                                                                         C=-np.transpose(A),
                                                                         b=-np.reshape(b, s3))
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
                self.P.variables.set_lower_bounds(i, -1*cplex.infinity)
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
