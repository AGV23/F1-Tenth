"""_summary_
This file contains functions for generating the various Matrices described in the paper
"""

from perception_data import Centreline
import numpy as np

def matAInv(N: np.int32):
    A = np.zeros((4*N, 4*N))
    for i in range(0, 4*N, 4):
        # Equation: x_n = a_n
        # i is address of a_n
        A[i][i] = 1 # coeff of a_n
    for i in range(1, 4*N, 4):
        # Equation: x_(n+1) = a_n + b_n + c_n + d_n
        # i is address of b_n
        A[i][i-1] = 1 # coeff of a_n
        A[i][i] = 1 # coeff of b_n
        A[i][i+1] = 1 # coeff of c_n
        A[i][i+2] = 1 # coeff of d_n
    for i in range(2, 4*N, 4):
        # Equation: 0 = x_1' - x_1' = b_(n-1) + 2c_(n-1) + 3d_(n-1) - b_n
        # i is address of c_n
        A[i][i-1] = -1 # coeff of b_n
        addr_A_N_1 = (i+4*N-6)%(4*N) # address of a_(n-1)
        A[i][addr_A_N_1 + 1] = 1 # coeff of b_(n-1)
        A[i][addr_A_N_1 + 2] = 2 # coeff of c_(n-1)
        A[i][addr_A_N_1 + 3] = 3 # coeff of d_(n-1)
    for i in range(3, 4*N, 4):
        # Equation: 0 = x_1'' - x_1'' = 2c_(n-1) + 6d_(n-1) - 2c_n
        # i is address of d_n
        A[i][i-1] = -2 # coeff of c_n
        addr_A_N_1 = (i+4*N-7)%(4*N) # address of a_(n-1)
        A[i][addr_A_N_1 + 2] = 2 # coeff of c_(n-1)
        A[i][addr_A_N_1 + 3] = 6 # coeff of d_(n-1)
    A_inv = np.linalg.inv(A)
    A_inv[np.isclose(A_inv, 0, atol=1e-15)] = 0
    return A_inv

def A_ex_comp(N: np.int32, component: np.int32):
    # e.g. returns A_ex,b for component = 1, A_ex,c for component = 2
    A_ex = np.zeros((N, 4*N), dtype=np.float64)
    for i in range(N): A_ex[i][4*i + component] = 1
    return A_ex

def q_comp(centreline: Centreline, component: np.int32):
    # e.g. q_x for component = 0, q_y for component = 1
    q = np.ndarray(4*centreline.N, dtype=np.float64)
    for i in range(0, 4*centreline.N, 4):
        q[i] = centreline.p[i//4, component]
        q[i+1] = centreline.p[(i//4 + 1)%centreline.N, component]
        q[i+2] = 0
        q[i+3] = 0
    return q

def M_comp(centreline: Centreline, component: np.int32):
    # e.g. M_x for component = 0, M_y for component = 1
    M = np.zeros((4*centreline.N, centreline.N), dtype=np.float64)
    for i in range(0, 4*centreline.N, 4):
        M[i, i//4] = centreline.n[i//4, component]
        M[i+1, (i//4 + 1)%centreline.N] = centreline.n[(i//4 + 1)%centreline.N, component]
    return M

def first_derivatives(centreline: Centreline, Ainv: np.ndarray, q: np.ndarray):
    A_ex_b = A_ex_comp(centreline.N, 1)
    return A_ex_b @ Ainv @ q

def matPxx(N: np.int32, x_dashed: np.ndarray, y_dashed: np.ndarray):
    return np.diag([ y_dashed[i]**2/(x_dashed[i]**2+y_dashed[i]**2)**3 for i in range(N) ])

def matPxy(N: np.int32, x_dashed: np.ndarray, y_dashed: np.ndarray):
    return np.diag([ -2*x_dashed[i]*y_dashed[i]/(x_dashed[i]**2+y_dashed[i]**2)**3 for i in range(N) ])

def matPyy(N: np.int32, x_dashed: np.ndarray, y_dashed: np.ndarray):
    return np.diag([ x_dashed[i]**2/(x_dashed[i]**2+y_dashed[i]**2)**3 for i in range(N) ])

def matrices_H_f(centreline: Centreline):
    # returns a tuple of the matrices H and f that define the QP
    Ainv = matAInv(centreline.N)
    A_ex_c = A_ex_comp(centreline.N, 2)
    q_x = q_comp(centreline, 0)
    q_y = q_comp(centreline, 1)
    x_d = first_derivatives(centreline, Ainv, q_x) # vector containing x_i'
    y_d = first_derivatives(centreline, Ainv, q_y) # vector containing y_i'

    centreline.calc_n(x_d, y_d)

    M_x = M_comp(centreline, 0)
    M_y = M_comp(centreline, 1)

    T_c = 2 * A_ex_c @ Ainv
    T_n_x = T_c @ M_x
    T_n_y = T_c @ M_y
    
    P_xx = matPxx(centreline.N, x_d, y_d)
    P_xy = matPxy(centreline.N, x_d, y_d)
    P_yy = matPyy(centreline.N, x_d, y_d)

    H_x = T_n_x.T @ P_xx @ T_n_x
    H_xy = T_n_y.T @ P_xy @ T_n_x
    H_y = T_n_y.T @ P_yy @ T_n_y
    H = 2*(H_x + H_xy + H_y)

    f_x = 2 * T_n_x.T @ P_xx.T @ T_c @ q_x
    f_xy = T_n_y.T @ P_xy.T @ T_c @ q_x + T_n_x.T @ P_xy.T @ T_c @ q_y
    f_y = 2 * T_n_y.T @ P_yy.T @ T_c @ q_y
    f = f_x + f_xy + f_y
    
    return H, f
