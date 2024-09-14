import numpy as np

class Centreline:
    """
    members:
    int N
    float p[N][2]
    float n[N][2]
    (x_i -> p[i, 0], y_i -> p[i, 1])
    """
    def __init__(self, N, p, half_track_width, vehicle_width):
        self.N = np.uint32(N)
        self.p = np.array(p, dtype=np.float64)
        self.half_w_tr = np.array(half_track_width, dtype=np.float64)
        self.w_veh = np.float64(vehicle_width)

    def calc_n(self, x_derivatives, y_derivatives):
        self.n = np.array([[y_d, -x_d] for i in range(self.N) if (x_d := x_derivatives[i], y_d := y_derivatives[i])], dtype=np.float64) # not normalized
        self.n /= np.repeat(np.linalg.norm(self.n, axis=1), 2, axis=0).reshape(-1, 2) # normalized here