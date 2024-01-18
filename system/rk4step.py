
"""
RK4 method for solving time-invariant ODEs
rk4step only takes one step

Arguments:
    f: function that takes x and returns dx/dt
    x: state, numpy array
    dt: time step
    *args: additional arguments/parameters for f
"""
def rk4step(f, x, dt, *args):
    k1 = f(x, *args)
    k2 = f(x + k1 * dt / 2.0, *args)
    k3 = f(x + k2 * dt / 2.0, *args)
    k4 = f(x + k3 * dt, *args)
    return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt / 6.0
