import numpy as np

def nonlin_quad_dyn(t, x, u, params):
    """
    Quadrotor dynamics with state
    x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r, W1, W2, W3, W4]

    phi, theta, psi : roll, pitch, yaw
    p, q, r         : body angular rates
    Wi              : actual rotor speeds
    u               : commanded rotor speeds
    """
    g  = params["g"]
    m  = params["m"]
    Jx = params["Jx"]
    Jy = params["Jy"]
    Jz = params["Jz"]
    kf = params["kf"]
    km = params["km"]
    l  = params["l"]
    tau_m = params["tau_m"]

    px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r, W1, W2, W3, W4 = x
    J = np.diag([Jx, Jy, Jz])

    cph, sph = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cps, sps = np.cos(psi), np.sin(psi)

    # body -> world rotation, ZYX = yaw-pitch-roll
    R = np.array([
        [cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph],
        [sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph],
        [-sth,    cth*sph,               cth*cph]
    ])

    # thrusts
    f1, f2, f3, f4 = kf*W1**2, kf*W2**2, kf*W3**2, kf*W4**2
    fT = f1 + f2 + f3 + f4

    # torques
    tau = np.array([
        l*(f2 - f4),
        l*(f3 - f1),
        km*(W1**2 - W2**2 + W3**2 - W4**2)
    ])

    omega = np.array([p, q, r])
    omega_hat = np.array([
        [0, -r,  q],
        [r,  0, -p],
        [-q, p,  0]
    ])

    # translational dynamics
    p_dot = np.array([vx, vy, vz])
    v_dot = g*np.array([0., 0., 1.]) - (fT/m)*R[:, 2]

    # Euler angle kinematics
    E = np.array([
        [1, sph*np.tan(theta), cph*np.tan(theta)],
        [0, cph,               -sph],
        [0, sph/cth,            cph/cth]
    ])
    ang_dot = E @ omega   # [phi_dot, theta_dot, psi_dot]

    # rigid body rotational dynamics
    omega_dot = np.linalg.solve(J, tau - omega_hat @ (J @ omega))

    # motor lag
    W = np.array([W1, W2, W3, W4])
    W_dot = (u - W) / tau_m

    return np.concatenate([p_dot, v_dot, ang_dot, omega_dot, W_dot])