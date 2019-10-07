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

