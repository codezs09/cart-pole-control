import sys
import numpy as np
from scipy.linalg import expm

pip3_control_path = "/home/deeproute/.local/lib/python3.8/site-packages"
sys.path.insert(0, pip3_control_path)
import control  # control system library

"""
    lsys_c2d: Continuous to discrete conversion
"""
def lsys_c2d(Ac, Bc, dt):
    n = Ac.shape[0]
    Ad = expm(Ac * dt)
    if np.linalg.det(Ac) != 0:
        Bd = np.matmul(np.matmul(np.linalg.inv(Ac), (Ad - np.eye(4))), Bc)
    else:
        M = np.zeros((n, n))
        Ac_pow = np.eye(n)
        factorial = 1.0
        dt_pow = dt
        for i in range(1, 5):
            M += Ac_pow / factorial * dt_pow
            Ac_pow = Ac_pow * Ac
            factorial *= (i + 1)
            dt_pow *= dt
        Bd = M @ Bc

    # debug
    if False:
        C = np.eye(n)
        D = np.zeros((n, 1))
        sys_c = control.ss(Ac, Bc, C, D)
        sys_d = control.c2d(sys_c, dt, method='zoh')
        Ad_check, Bd_check, _, _ = control.ssdata(sys_d)
        print("-----------------------------------------------------------")
        print("Ad: ", Ad)
        print("Ad_check: ", Ad_check)
        print("Bd: ", Bd)
        print("Bd_check: ", Bd_check)

    return Ad, Bd
