#!/usr/bin/env python3
"""
Contains a model of a 3DOF marine ship. Supplies parameter values,
a function for the full nonlinear dynamic, and functions for
jacobians of said dynamic. Running this script as __main__ will
present a quick open-loop simulation, comparing the nonlinear
dynamics to the linearized (about every time step) dynamics.

State is: [x_world_position (m),
           y_world_position (m),
           heading_angle_from_x (rad),
           x_body_velocity (m/s),
           y_body_velocity (m/s),
           yaw_rate (rad/s)]

Input is: [x_body_force (N),
           y_body_force (N),
           z_torque (N*m)]

See:
T. I. Fossen, Handbook of Marine Craft Hydrodynamics
and Motion Control. Wiley, 2011. Chapter 13.

Partial code taken from https://github.com/jnez71/misc
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
npl = np.linalg

# PHYSICAL PARAMETERS

# specific mass, inertia and dimensions
m = 6.342  # kg
xg = 0  # COM x coord in bocy fixed frame [m]
Iz =  4  #(w**2 + l**2)/12  # mass moment of intertia of a box (l,w,h)

# Fluid inertial effects
wm_xu = -0.025*m  # kg          # These expressions are just an okay starting point
wm_yv = -0.25*m  # kg           # if all you somewhat know are m, Iz, and xg
wm_yr = -0.25*m*xg  # kg*m
wm_nr = -0.25*Iz  # kg*m**2

# Drag
d_xuu = -3.5 #0.25 * wm_xu  # N/(m/s)**2
d_yvv = -4 #0.25 * wm_yv  # N/(m/s)**2
d_nrr = -4 #0.25 * (wm_nr + wm_yr)  # (N*m)/(rad/s)**2

# Cross-flow
d_yrr = 0.25 * wm_yr  # N/(rad/s)**2
d_yrv = 0.25 * wm_yr  # N/(m*rad/s**2)
d_yvr = 0.25 * wm_yv  # N/(m*rad/s**2)
d_nvv = 0.25 * d_yvv  # (N*m)/(m/s)**2
d_nrv = 0.25 * d_yrv  # (N*m)/(m*rad/s**2)
d_nvr = 0.25 * (wm_nr + wm_yv)  # (N*m)/(m*rad/s**2)

# EQUATIONS OF MOTION

# Inertial matrix, independent of state
M = np.array([
              [m - wm_xu,            0,            0],
              [        0,    m - wm_yv, m*xg - wm_yr],
              [        0, m*xg - wm_yr,   Iz - wm_nr]
            ])
Minv = npl.inv(M)


def dynamics(q, u):
    """
    qdot = dynamics(q, u)

    """
    # Centripetal-coriolis matrix
    C = np.array([
                  [                                     0,                0, (wm_yr - m*xg)*q[5] + (wm_yv - m)*q[4]],
                  [                                     0,                0,                       (m - wm_xu)*q[3]],
                  [(m*xg - wm_yr)*q[5] + (m - wm_yv)*q[4], (wm_xu - m)*q[3],                                      0]
                ])

    # Drag matrix
    D = np.array([
                  [-d_xuu*abs(q[3]),                                    0,                                    0],
                  [               0, -(d_yvv*abs(q[4]) + d_yrv*abs(q[5])), -(d_yvr*abs(q[4]) + d_yrr*abs(q[5]))],
                  [               0, -(d_nvv*abs(q[4]) + d_nrv*abs(q[5])), -(d_nvr*abs(q[4]) + d_nrr*abs(q[5]))]
                ])

    R = get_R(q)

    # M*vdot + C*v + D*v = u  and  pdot = R*v
    return np.concatenate((R.dot(q[3:]), Minv.dot(u - (C + D).dot(q[3:]))))


def A(q):
    """
    Jacobian of f with respect to q.

    """
    return np.array([
                     [ 0, 0, - q[4]*np.cos(q[2]) - q[3]*np.sin(q[2]),                                                                                                                                                                                                                                             np.cos(q[2]),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -np.sin(q[2]),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0],
                     [ 0, 0,   q[3]*np.cos(q[2]) - q[4]*np.sin(q[2]),                                                                                                                                                                                                                                             np.sin(q[2]),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          np.cos(q[2]),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0],
                     [ 0, 0,                         0,                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 1],
                     [ 0, 0,                         0,                                                                                                                                                                                                     (d_xuu*abs(q[3]) + d_xuu*q[3]*np.sign(q[3]))/(m - wm_xu),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    (m*q[5] - q[5]*wm_yv)/(m - wm_xu),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            (m*q[4] - 2*q[5]*wm_yr - q[4]*wm_yv + 2*m*q[5]*xg)/(m - wm_xu)],
                     [ 0, 0,                         0, -(q[5]*wm_yr**2 + m**2*q[5]*xg**2 - Iz*m*q[5] + Iz*q[5]*wm_xu + m*q[5]*wm_nr - q[5]*wm_nr*wm_xu - q[4]*wm_yr*wm_xu + q[4]*wm_yr*wm_yv - 2*m*q[5]*wm_yr*xg + m*q[4]*wm_xu*xg - m*q[4]*wm_yv*xg)/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg), -(d_nrv*wm_yr*abs(q[5]) - d_yrv*wm_nr*abs(q[5]) + d_nvv*wm_yr*abs(q[4]) - d_yvv*wm_nr*abs(q[4]) - q[3]*wm_yr*wm_xu + q[3]*wm_yr*wm_yv + Iz*d_yrv*abs(q[5]) + Iz*d_yvv*abs(q[4]) + m*q[3]*wm_xu*xg - m*q[3]*wm_yv*xg + Iz*d_yvr*q[5]*np.sign(q[4]) + Iz*d_yvv*q[4]*np.sign(q[4]) - d_nrv*m*xg*abs(q[5]) - d_nvv*m*xg*abs(q[4]) + d_nvr*q[5]*wm_yr*np.sign(q[4]) - d_yvr*q[5]*wm_nr*np.sign(q[4]) + d_nvv*q[4]*wm_yr*np.sign(q[4]) - d_yvv*q[4]*wm_nr*np.sign(q[4]) - d_nvr*m*q[5]*xg*np.sign(q[4]) - d_nvv*m*q[4]*xg*np.sign(q[4]))/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg), -(q[3]*wm_yr**2 + d_nrr*wm_yr*abs(q[5]) - d_yrr*wm_nr*abs(q[5]) + d_nvr*wm_yr*abs(q[4]) - d_yvr*wm_nr*abs(q[4]) + m**2*q[3]*xg**2 - Iz*m*q[3] + Iz*q[3]*wm_xu + m*q[3]*wm_nr - q[3]*wm_nr*wm_xu + Iz*d_yrr*abs(q[5]) + Iz*d_yvr*abs(q[4]) - 2*m*q[3]*wm_yr*xg + Iz*d_yrr*q[5]*np.sign(q[5]) + Iz*d_yrv*q[4]*np.sign(q[5]) - d_nrr*m*xg*abs(q[5]) - d_nvr*m*xg*abs(q[4]) + d_nrr*q[5]*wm_yr*np.sign(q[5]) - d_yrr*q[5]*wm_nr*np.sign(q[5]) + d_nrv*q[4]*wm_yr*np.sign(q[5]) - d_yrv*q[4]*wm_nr*np.sign(q[5]) - d_nrr*m*q[5]*xg*np.sign(q[5]) - d_nrv*m*q[4]*xg*np.sign(q[5]))/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg)],
                     [ 0, 0,                         0,                                             (q[4]*wm_yv**2 + m*q[4]*wm_xu - m*q[4]*wm_yv - q[5]*wm_yr*wm_xu + q[5]*wm_yr*wm_yv - q[4]*wm_xu*wm_yv + m*q[5]*wm_xu*xg - m*q[5]*wm_yv*xg)/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg),                (q[3]*wm_yv**2 - d_nrv*m*abs(q[5]) - d_nvv*m*abs(q[4]) + d_nrv*wm_yv*abs(q[5]) - d_yrv*wm_yr*abs(q[5]) + d_nvv*wm_yv*abs(q[4]) - d_yvv*wm_yr*abs(q[4]) + m*q[3]*wm_xu - m*q[3]*wm_yv - q[3]*wm_xu*wm_yv + d_yrv*m*xg*abs(q[5]) + d_yvv*m*xg*abs(q[4]) - d_nvr*m*q[5]*np.sign(q[4]) - d_nvv*m*q[4]*np.sign(q[4]) + d_nvr*q[5]*wm_yv*np.sign(q[4]) - d_yvr*q[5]*wm_yr*np.sign(q[4]) + d_nvv*q[4]*wm_yv*np.sign(q[4]) - d_yvv*q[4]*wm_yr*np.sign(q[4]) + d_yvr*m*q[5]*xg*np.sign(q[4]) + d_yvv*m*q[4]*xg*np.sign(q[4]))/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg),                                      -(d_nrr*m*abs(q[5]) + d_nvr*m*abs(q[4]) - d_nrr*wm_yv*abs(q[5]) + d_yrr*wm_yr*abs(q[5]) - d_nvr*wm_yv*abs(q[4]) + d_yvr*wm_yr*abs(q[4]) + q[3]*wm_yr*wm_xu - q[3]*wm_yr*wm_yv - m*q[3]*wm_xu*xg + m*q[3]*wm_yv*xg - d_yrr*m*xg*abs(q[5]) - d_yvr*m*xg*abs(q[4]) + d_nrr*m*q[5]*np.sign(q[5]) + d_nrv*m*q[4]*np.sign(q[5]) - d_nrr*q[5]*wm_yv*np.sign(q[5]) + d_yrr*q[5]*wm_yr*np.sign(q[5]) - d_nrv*q[4]*wm_yv*np.sign(q[5]) + d_yrv*q[4]*wm_yr*np.sign(q[5]) - d_yrr*m*q[5]*xg*np.sign(q[5]) - d_yrv*m*q[4]*xg*np.sign(q[5]))/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg)]
                   ])


# Jacobian of f with respect to u, independent of state
B = np.array([ 
              [             0,                                                                                             0,                                                                                             0],
              [             0,                                                                                             0,                                                                                             0],
              [             0,                                                                                             0,                                                                                             0],
              [ 1/(m - wm_xu),                                                                                             0,                                                                                             0],
              [             0,   -(Iz - wm_nr)/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg), -(wm_yr - m*xg)/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg)],
              [             0, -(wm_yr - m*xg)/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg),    -(m - wm_yv)/(m**2*xg**2 - Iz*m + Iz*wm_yv + m*wm_nr - wm_nr*wm_yv + wm_yr**2 - 2*m*wm_yr*xg)]
            ])


def unwrap(ang):
    "Returns an angle on [-pi, pi]"
    return np.mod(ang+np.pi, 2*np.pi) - np.pi

def qplus(q, dq):
    "Adds a perturbation to a state, q [+] dq"
    qp = q + dq
    qp[2] = unwrap(qp[2])
    return qp

def qminus(ql, qr):
    "Subtracts two states, ql [-] qr"
    dq = ql - qr
    dq[2] = unwrap(dq[2])
    return dq

def get_R(q):
    """ Rotation matrix (orientation, converts body to world)"""
    return np.array([
        [np.cos(q[2]), -np.sin(q[2]), 0],
        [np.sin(q[2]),  np.cos(q[2]), 0],
        [0,             0, 1]
    ])

# SIMULATION
if __name__ == "__main__":

    # Simulation duration, timestep and animation parameters
    t0 = 0
    T = 20  # s
    dt = 0.001

    framerate = 20  # fps
    speedup = 100  # kinda makes the playback a little faster
    store_data = False  # should data be stored into a .mat?
    outline_path = True  # show path outline on animation?

    # Initial condition
    # [m, m, rad, m/s, m/s, rad/s]
    x0 = -1
    q0 = np.array([x0, 0, 0, 0, 0, 0], dtype=np.float64)
    q = np.copy(q0)
    qlin = np.copy(q0)
    u = np.array([0, 0, 0], dtype=np.float64)  # [N, N, N*m]

    # Define time domain
    t_arr = np.arange(t0, T, dt)

    # Preallocate results memory
    q_history = np.zeros((len(t_arr), len(q)))
    qref_history = np.zeros((len(t_arr), int(len(q)/2)))
    qlin_history = np.zeros((len(t_arr), len(qlin)))
    u_history = np.zeros((len(t_arr), int(len(q)/2)))
    u_world_history = np.zeros((len(t_arr), int(len(q)/2)))

    # Integrate dynamics using first-order forward stepping
    for i, t in enumerate(t_arr):

        # Tension input
        R = get_R(q)

        u = np.array([np.cos(t), np.sin(t), 0])

        # Record this instant
        q_history[i] = q
        qref_history[i] = u 
        qlin_history[i] = qlin
        u_history[i] = u
        u_world_history[i] = R.dot(u)

        # Step forward, qnext = qlast + qdot*dt
        q = qplus(q, dynamics(q, u)*dt)
        qlin = qplus(qlin, (A(q).dot(qminus(qlin, q)) + B.dot(u))*dt)

    # Figure for individual results
    fig1 = plt.figure()
    fig1.suptitle('State Evolution', fontsize=20)
    fig1rows = 2
    fig1cols = 5

    # Plot x position
    ax = fig1.add_subplot(fig1rows, fig1cols, 1)
    ax.set_title('X Position (m)', fontsize=16)
    ax.plot(t_arr, q_history[:, 0], 'g', label="nonlinear")
    ax.plot(t_arr, qlin_history[:, 0], 'k', label="linearized")
    ax.plot(t_arr, qref_history[:, 0], 'b', label="drone")
    ax.grid(True)
    ax.legend()

    # Plot y position
    ax = fig1.add_subplot(fig1rows, fig1cols, 2)
    ax.set_title('Y Position (m)', fontsize=16)
    ax.plot(t_arr, q_history[:, 1], 'g',
            t_arr, qlin_history[:, 1], 'k',
            t_arr, qref_history[:, 1], 'b')
    ax.grid(True)

    # Plot yaw position
    ax = fig1.add_subplot(fig1rows, fig1cols, 3)
    ax.set_title('Heading (deg)', fontsize=16)
    ax.plot(t_arr, np.rad2deg(q_history[:, 2]), 'g',
             t_arr, np.rad2deg(qlin_history[:, 2]), 'k')
    ax.grid(True)

    # Plot control efforts
    ax = fig1.add_subplot(fig1rows, fig1cols, 4)
    ax.set_title('Body-fixed Wrench (N, N, N*m)', fontsize=16)
    ax.plot(t_arr, u_history[:, 0], 'b', label='x [N]')
    ax.plot(t_arr, u_history[:, 1], 'g', label='y [N]')
    ax.plot(t_arr, u_history[:, 2], 'r', label='z [N m]')
    ax.legend()
    ax.grid(True)

    # Plot x velocity
    ax = fig1.add_subplot(fig1rows, fig1cols, 6)
    ax.set_title('Surge (m/s)', fontsize=16)
    ax.plot(t_arr, q_history[:, 3], 'g',
             t_arr, qlin_history[:, 3], 'k')
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    # Plot y velocity
    ax = fig1.add_subplot(fig1rows, fig1cols, 7)
    ax.set_title('Sway (m/s)', fontsize=16)
    ax.plot(t_arr, q_history[:, 4], 'g',
             t_arr, qlin_history[:, 4], 'k')
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    # Plot yaw velocity
    ax = fig1.add_subplot(fig1rows, fig1cols, 8)
    ax.set_title('Yaw (deg/s)', fontsize=16)
    ax.plot(t_arr, np.rad2deg(q_history[:, 5]), 'g',
             t_arr, np.rad2deg(qlin_history[:, 5]), 'k')
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    # Plot norm linearization errors
    ax = fig1.add_subplot(fig1rows, fig1cols, 5)
    ax.set_title('Norm Linearization Error', fontsize=16)
    ax.plot(t_arr, npl.norm([qminus(q_history[i], qlin_history[i]) for i in range(len(t_arr))], axis=1), 'k')
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    # Dist drone and boat
    ax = fig1.add_subplot(fig1rows, fig1cols, 9)
    ax.set_title('Distance drone and boat (m)', fontsize=16)
    ax.plot(t_arr, qref_history[:, 0] - q_history[:, 0], 'b', label='x [m]')
    ax.plot(t_arr, qref_history[:, 1] - q_history[:, 1], 'g', label='y [m]')
    ax.set_xlabel('Time (s)')
    plt.legend()
    ax.grid(True)

    # Figure for animation
    fig6 = plt.figure()
    fig6.suptitle('Evolution')
    ax6 = fig6.add_subplot(1, 1, 1)
    ax6.set_xlabel('X (m)')
    ax6.set_ylabel('Y (m)')
    ax6.set_aspect('equal', 'datalim')
    ax6.grid(True)

    # Points and lines for representing positions and headings
    pthick = 100
    lthick = 2
    llen = 2.5
    p = ax6.scatter(q_history[0, 0], q_history[0, 1], color='k', s=pthick)
    h = ax6.plot([q_history[0, 0], q_history[0, 0] + llen*np.cos(q_history[0, 2])],
                 [q_history[0, 1], q_history[0, 1] + llen*np.sin(q_history[0, 2])], color='k', linewidth=lthick)
    pref = ax6.scatter(qref_history[0, 0], qref_history[0, 1], color='b', s=pthick)
    href = ax6.plot([qref_history[0, 0], qref_history[0, 0] - 0.8*llen*u_world_history[0, 0]],
                    [qref_history[0, 1], qref_history[0, 1] - 0.8*llen*u_world_history[0, 1]], color='r', linewidth=lthick)

    # Plot entirety of actual trajectory
    if outline_path:
        ax6.plot(q_history[:, 0], q_history[:, 1], 'k--',
                 qref_history[:, 0], qref_history[:, 1], 'g--')

    # Function for updating the animation frame

    def update_ani(arg, ii=[0]):

        i = ii[0]  # don't ask...

        if np.isclose(t_arr[i], np.around(t_arr[i], 1)):
            fig6.suptitle('Evolution (Time: {})'.format(t_arr[i]), fontsize=24)

        p.set_offsets((q_history[i, 0], q_history[i, 1]))
        h[0].set_data([q_history[i, 0], q_history[i, 0] + llen*np.cos(q_history[i, 2])],
                      [q_history[i, 1], q_history[i, 1] + llen*np.sin(q_history[i, 2])])
        pref.set_offsets((qref_history[i, 0], qref_history[i, 1]))
        href[0].set_data([qref_history[i, 0], qref_history[i, 0] - 0.8*llen*u_world_history[i, 0]],
                         [qref_history[i, 1], qref_history[i, 1] - 0.8*llen*u_world_history[i, 1]])

        ii[0] += int(1 / (dt * framerate))
        if ii[0] >= len(t_arr):
            print("Resetting animation!")
            ii[0] = 0

        else:
            return [p, h, pref, href]

    # Run animation
    ani = animation.FuncAnimation(
        fig6, func=update_ani, interval=dt*1000/speedup)
    plt.show()

#    # Store data
    if store_data:
        from scipy.io import savemat
        filename = 'data.m'
        data = {'time': t_arr,
                'north': q_history[:, 0],
                'east': q_history[:, 1],
                'heading': q_history[:, 2],
                'surge': q_history[:, 3],
                'sway': q_history[:, 4],
                'yawrate': q_history[:, 5],
                'north_des': qref_history[:, 0],
                'east_des': qref_history[:, 1],
                'heading_des': qref_history[:, 2],
                'surge_des': qref_history[:, 3],
                'sway_des': qref_history[:, 4],
                'yawrate_des': qref_history[:, 5],
                'surge_force': u_history[:, 0],
                'sway_force': u_history[:, 1],
                'yaw_torque': u_history[:, 2],
                }
        savemat(filename, data)
        print('Data saved!\n')
