import cvxpy as cp
from perception_data import Centreline
from matrices import matrices_H_f

def solve_for_alpha(centreline: Centreline):
    P, q = matrices_H_f(centreline)

    N = centreline.N
    alpha = cp.Variable(N)

    alpha_max = (centreline.half_w_tr - centreline.w_veh/2)
    alpha_min = -(centreline.half_w_tr - centreline.w_veh/2)

    # Define and solve the CVXPY problem.
    P = (P+P.T)/2
    prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(alpha, cp.psd_wrap(P)) + q.T @ alpha),
                    [alpha <= alpha_max,
                    alpha >= alpha_min])
    prob.solve()

    # # Print result.
    # print("\nThe optimal value is", prob.value)
    # print("A solution alpha is")
    # print(alpha.value)
    # print("A dual solution corresponding to the inequality constraints is")
    # print(prob.constraints[0].dual_value)
    return alpha.value
