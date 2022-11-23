#!/usr/bin/env python3
"""
Contains a model of a 3DOF marine ship. Supplies parameter values,
a function for the full nonlinear dynamic. Running this script as
__main__ will present a quick open-loop simulation.

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

path_type = 'data'  # 'line' or 'data'

if path_type == 'data':
    # Trajectory
    echo_3_051722 = pd.read_csv('./traj_data/force_traj_3.csv')
    echo_6_051722 = pd.read_csv('./traj_data/force_traj_6.csv')
    echo_9_051722 = pd.read_csv('./traj_data/force_traj_9.csv')
    echo_12_051722 = pd.read_csv('./traj_data/force_traj_12.csv')
    drone_3 = pd.read_csv('./traj_data/drone_traj_3.csv')
    drone_6 = pd.read_csv('./traj_data/drone_traj_6.csv')
    drone_9 = pd.read_csv('./traj_data/drone_traj_9.csv')
    drone_12 = pd.read_csv('./traj_data/drone_traj_12.csv')
    echos = [echo_3_051722, echo_6_051722, echo_9_051722, echo_12_051722]
    drones = [drone_3, drone_6, drone_9, drone_12]

# PHYSICAL PARAMETERS

# Offset on the tension application point [m]
off_x = 0.1905  # measured from the bathydrone vehicle
off_z = 0.1016
r = np.array([off_x, 0, off_z])
# height of drone above water
hd = 20*0.681818  # feet to meters
le = 31*0.681818  # Length of rope
proj_le = np.sqrt(le**2-hd**2)

# Bathydrone specific mass, inertia and dimensions
m = 6.342  # kg
xg = 0  # COM x coord in bocy fixed frame [m]
Iz =  4  # mass moment of intertia from SolidWorks 
tension_magnitudes = [9, 11, 18, 27.5]

# Fluid inertial effects
wm_xu = -6*m  # kg        Increase to damp more the yaw wrt to drone 
wm_yv = -0.5*m  # kg       #    
wm_yr = -0.5*m*xg  # kg*m
wm_nr = -0.25*Iz  # kg*m**2  #

# Drag
d_xuu = -5.75 #0.25 * wm_xu  # N/(m/s)**2
d_yvv = -7 #0.25 * wm_yv  # N/(m/s)**2
d_nrr = -7 #0.25 * (wm_nr + wm_yr)  # (N*m)/(rad/s)**2

# Cross-flow
d_yrr = 0.25 * wm_yr  # N/(rad/s)**2
d_yrv = 0.25 * wm_yr  # N/(m*rad/s**2)
d_yvr = 0.25 * wm_yv  # N/(m*rad/s**2)
d_nvv = 0.25 * d_yvv  # (N*m)/(m/s)**2  #
d_nrv = 0.25 * d_yrv  # (N*m)/(m*rad/s**2)  #
d_nvr = 0.25 * (wm_nr + wm_yv)  # (N*m)/(m*rad/s**2)  #

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
    Dynamic model of the boat. Returns the derivatives of the state q based on the control input u
    Input: 
        q: State of the boat. Vector of position, orientation and velocites
        u: control input
    Output:
        q_dot: time rate of change of the state. Velocities and acceleration 
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
    dyn = np.concatenate((R.dot(q[3:]), Minv.dot(u - (C + D).dot(q[3:]))))
    return dyn

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

def get_boat_position(t, ii):
    """
    Return the position of the boat in a point in time
    Path types:
        'line' = empty
        'data' = get the trajectory from a csv
            for data it finds the index of the data time that most
            closely match the simulation time
    """
    if path_type == 'line':
        return np.array([0, 0, 0])
    elif path_type == 'data':
        t_i = df_dr['time'].iloc[ii]
        while t_i <= t:
            t_i = df_dr['time'].iloc[ii]
            ii += 1
        x = df_bt['X_m'].iloc[ii]
        y = df_bt['Y_m'].iloc[ii]
        return np.array([x, y, 0]), ii

def get_drone_position(t, hd, ii):
    """
    Return the position of the drone in a point in time,
    depending on the desired velocity
    Path types:
        'line' = straight line
        'data' = get the trajectory from a csv
            for data it finds the index of the data time that most
            closely match the simulation time
    """
    if path_type == 'line':
        return np.array([0.7*t, 0.7*t, hd]), 0
    elif path_type == 'data':
        t_i = df_dr['time'].iloc[ii]
        while t_i <= t:
            t_i = df_dr['time'].iloc[ii]
            ii += 1
        x = df_dr['X_m'].iloc[ii]
        y = df_dr['Y_m'].iloc[ii]
        return np.array([x, y, hd]), ii

def get_R(q):
    """ Rotation matrix (orientation, converts body to world)"""
    return np.array([
        [np.cos(q[2]), -np.sin(q[2]), 0],
        [np.sin(q[2]),  np.cos(q[2]), 0],
        [0,             0, 1]
    ])

# SIMULATION
if __name__ == "__main__":

    if path_type == 'data':
        # Choose one option from the datasets (drones and echos)
        df_dr = drones[1]
        df_bt = echos[1]

        # Fix delta in time
        #kk = np.argmax(df_dr['Y_m'].to_numpy() <= df_bt['Y_m'].to_numpy()[0])
        #tt = df_dr["time"].to_numpy()[80]
        df_dr["time"] = df_dr["time"] - df_dr["time"].to_numpy()[76]
        df_dr = df_dr.drop(df_dr.index[range(0, 76)])
    ten_mag = tension_magnitudes[1]  # choose magnitude depending on the drones dataset 
    i_last = 0
    ii_last = 0
    t_last = 0

    # Simulation duration, timestep and animation parameters
    t0 = 0
    if path_type == 'line':
        T = 60  # s
        dt = 0.001
    elif path_type == 'data':
        T = np.min([df_bt['time1'].to_numpy()[-1],df_dr['time'].to_numpy()[-1]])
        dt = 0.01

    # Matplotlib Animation Parameters 
    framerate = 20  # fps
    speedup = 100  # kinda makes the playback a little faster
    store_data = False  # should data be stored into a .mat?
    outline_path = True  # show path outline on animation?

    # Initial condition
    # [m, m, rad, m/s, m/s, rad/s]
    x0 = -14
    if path_type == 'line':
        q0 = np.array([x0, -np.sqrt(proj_le**2 - x0**2),
                       0, 0, 0, 0], dtype=np.float64)
    elif path_type == 'data':
        jj = np.argmax(df_dr['time'].to_numpy() >= t0)
        #q0 = np.array([x0 + df_dr['X_m'].iloc[jj],
        #        np.sqrt(proj_le**2 - x0**2) + df_dr['Y_m'].iloc[jj],
        #        0, 0, 0, 0], dtype=np.float64)
        q0 = np.array([df_bt['X_m'].iloc[jj], df_bt['Y_m'].iloc[jj],
                0, 0, 0, 0], dtype=np.float64)
    q = np.copy(q0)
    u = np.array([0, 0, 0], dtype=np.float64)  # [N, N, N*m]
    dr = np.array([0, 0, 0], dtype=np.float64)  # [m, m, rad]

    # Define time domain
    t_arr = np.arange(t0, T, dt)

    # Preallocate results memory
    q_history = np.zeros((len(t_arr), len(q)))
    dr_history = np.zeros((len(t_arr), int(len(q)/2)))
    bt_history = np.zeros((len(t_arr), int(len(q)/2)))
    u_history = np.zeros((len(t_arr), int(len(q)/2)))
    u_world_history = np.zeros((len(t_arr), int(len(q)/2)))
    head_dr = np.zeros((len(t_arr)))

    # Integrate dynamics using first-order forward stepping
    for i, t in enumerate(t_arr):

        R = get_R(q)
        # Rope length constraint
        if npl.norm(q[:2] - dr[:2]) <= proj_le or t == 0.0:
            dr, i_last = get_drone_position(t_last, hd, i_last)
            t_last += dt
        
        app_point = np.array([q[0], q[1], 0]) + R.dot(r) # world coord
        diff_pos = dr - app_point
        diff_pos = diff_pos/npl.norm(diff_pos)

        ten = ten_mag*diff_pos
        ten_body = R.T.dot(ten)
        moments = np.cross(r, ten_body)
        u[:2] = ten_body[:2]
        u[2] = moments[2]  # apply moment about z
        ten[2] = moments[2]
        
        if npl.norm(q[:2] - dr[:2]) < proj_le-2:
            u = np.array([0, 0, 0])

        br, ii_last = get_boat_position(t, ii_last)

        #if q[1] <= df_bt['Y_m'].to_numpy()[0]:
        #    print(i)
        # Record this instant

        q_history[i] = q
        dr_history[i] = dr
        u_history[i] = u
        u_world_history[i] = ten
        head_dr[i] = np.degrees(np.arctan2(diff_pos[1],diff_pos[0]))
        bt_history[i] = br 

        # Step forward, qnext = qlast + qdot*dt
        q = qplus(q, dynamics(q, u)*dt)
    
    ## PLOTS

    # Figure for individual results
    fig1 = plt.figure()
    fig1.suptitle('State Evolution', fontsize=20)
    fig1rows = 2
    fig1cols = 5

    # Plot x position
    ax = fig1.add_subplot(fig1rows, fig1cols, 1)
    ax.set_title('X Position (m)', fontsize=16)
    ax.plot(t_arr, q_history[:, 0], 'g', label="nonlinear")
    ax.plot(t_arr, dr_history[:, 0], 'b', label="drone")
    if path_type == 'data': ax.plot(t_arr, bt_history[:,0], 'k', label='boat')
    ax.grid(True)
    ax.legend()

    # Plot y position
    ax = fig1.add_subplot(fig1rows, fig1cols, 2)
    ax.set_title('Y Position (m)', fontsize=16)
    ax.plot(t_arr, q_history[:, 1], 'g',
            t_arr, dr_history[:, 1], 'b')
    if path_type == 'data': ax.plot(t_arr, bt_history[:,1], 'k')
    ax.grid(True)

    # Plot orientation
    ax = fig1.add_subplot(fig1rows, fig1cols, 3)
    ax.set_title('Heading (deg)', fontsize=16)
    ax.plot(t_arr, np.rad2deg(q_history[:, 2]), 'g',)
    ax.plot(t_arr, head_dr, 'k', label='heading from boat to drone')
    ax.grid(True)
    ax.legend()

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
    ax.plot(t_arr, q_history[:, 3], 'g',)
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    # Plot y velocity
    ax = fig1.add_subplot(fig1rows, fig1cols, 7)
    ax.set_title('Sway (m/s)', fontsize=16)
    ax.plot(t_arr, q_history[:, 4], 'g',)
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    # Plot yaw velocity
    ax = fig1.add_subplot(fig1rows, fig1cols, 8)
    ax.set_title('Yaw (deg/s)', fontsize=16)
    ax.plot(t_arr, np.rad2deg(q_history[:, 5]), 'g',)
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    # Wrench in world frame
    ax = fig1.add_subplot(fig1rows, fig1cols, 5)
    ax.set_title('World Wrench [N, N, Nm]', fontsize=16)
    ax.plot(t_arr, u_world_history[:, 0], 'b', label='x [N]')
    ax.plot(t_arr, u_world_history[:, 1], 'g', label='y [N]')
    ax.plot(t_arr, u_world_history[:, 2], 'r', label='z [N m]')
    ax.grid(True)

    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True)

    # Dist drone and boat
    ax = fig1.add_subplot(fig1rows, fig1cols, 9)
    ax.set_title('Distance drone and boat (m)', fontsize=16)
    ax.plot(t_arr, dr_history[:, 0] - q_history[:, 0], 'b', label='x [m]')
    ax.plot(t_arr, dr_history[:, 1] - q_history[:, 1], 'g', label='y [m]')
    ax.set_xlabel('Time (s)')
    plt.legend()
    ax.grid(True)

    # Dist norm drone and boat
    ax = fig1.add_subplot(fig1rows, fig1cols, 10)
    ax.set_title('Norm of Distance (m)', fontsize=16)
    ax.plot(t_arr, np.sqrt((q_history[:, 0] - dr_history[:, 0])**2 + (
        q_history[:, 1] - dr_history[:, 1])**2), 'k')
    ax.plot([t_arr[0], t_arr[-1]], [proj_le, proj_le], '-r', label='rope length')
    ax.set_xlabel('Time (s)')
    plt.legend()
    ax.grid(True)
    plt.show()


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
    pref = ax6.scatter(dr_history[0, 0], dr_history[0, 1], color='b', s=pthick)
    href = ax6.plot([dr_history[0, 0], dr_history[0, 0] - 0.8*llen*u_world_history[0, 0]],
                    [dr_history[0, 1], dr_history[0, 1] - 0.8*llen*u_world_history[0, 1]], color='r', linewidth=lthick)
    pact = ax6.scatter(bt_history[0, 0], bt_history[0, 1], color='c', s=pthick)

    # Plot entirety of actual trajectory
    if outline_path:
        ax6.plot(q_history[:, 0], q_history[:, 1], 'k--',
                 dr_history[:, 0], dr_history[:, 1], 'g--',
                 bt_history[:, 0], bt_history[:, 1], 'm--')

    # Function for updating the animation frame

    def update_ani(arg, ii=[0]):

        i = ii[0]  # don't ask...

        if np.isclose(t_arr[i], np.around(t_arr[i], 1)):
            fig6.suptitle('Evolution (Time: {})'.format(t_arr[i]), fontsize=24)

        p.set_offsets((q_history[i, 0], q_history[i, 1]))
        h[0].set_data([q_history[i, 0], q_history[i, 0] + llen*np.cos(q_history[i, 2])],
                      [q_history[i, 1], q_history[i, 1] + llen*np.sin(q_history[i, 2])])
        pref.set_offsets((dr_history[i, 0], dr_history[i, 1]))
        href[0].set_data([dr_history[i, 0], dr_history[i, 0] - 0.8*llen*u_world_history[i, 0]],
                         [dr_history[i, 1], dr_history[i, 1] - 0.8*llen*u_world_history[i, 1]])
        pact.set_offsets((bt_history[i, 0], bt_history[i, 1]))

        ii[0] += int(1 / (dt * framerate))
        if ii[0] >= len(t_arr):
            print("Resetting animation!")
            ii[0] = 0

        else:
            return [p, h, pref, href, pact]

    # Run animation
    ani = animation.FuncAnimation(
        fig6, func=update_ani, interval=dt*1000/speedup)
    plt.show()

#    # Store data
    if store_data:
        from scipy.io import savemat
        filename = 'data_{}_{}s'.format(path_type, T)
        data = {'time': t_arr,
                'north': q_history[:, 0],
                'east': q_history[:, 1],
                'heading': q_history[:, 2],
                'surge': q_history[:, 3],
                'sway': q_history[:, 4],
                'yawrate': q_history[:, 5],
                'north_des': dr_history[:, 0],
                'east_des': dr_history[:, 1],
                'heading_des': dr_history[:, 2],
                'surge_des': dr_history[:, 3],
                'sway_des': dr_history[:, 4],
                'yawrate_des': dr_history[:, 5],
                'surge_force': u_history[:, 0],
                'sway_force': u_history[:, 1],
                'yaw_torque': u_history[:, 2],
                }
        savemat(filename, data)
        print('Data saved!\n')

    plt.rcParams['text.usetex'] = True
    plt.rcParams.update({'font.size': 13})
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(1, 1, 1)
    ax2.plot(q_history[:, 0], q_history[:, 1], '--k', label='Boat Traj.')
    ax2.plot(q_history[0,0],q_history[0,1], 'ok', label='Boat Start Point')
    #ax2.plot(q_history[-1,0],q_history[-1,1], 'sk', label='Boat End Point')
    ax2.plot(dr_history[:,0],dr_history[:,1], '--b', label='Drone Traj.')
    ax2.plot(dr_history[0,0],dr_history[0,1], 'ob', label='Drone Start Point')
    #ax2.plot(dr_history[-1,0],dr_history[-1,1], 'sb', label='drone final position')
    ax2.set_xlabel('$x_I$ (m)')
    ax2.set_ylabel('$y_I$ (m)')
    ax2.grid(True)
    plt.legend()
    plt.show()
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(1, 1, 1)
    ax3.plot(q_history[:,0],q_history[:,1], '--k', label='Boat Traj. in Sim.')
    ax3.plot(bt_history[:,0],bt_history[:,1], '-r', label='Boat Traj. in Exp.')
    ax3.plot(q_history[0,0],q_history[0,1], 'ok', label='Start Point (Sim. and Exp.)')
    ax3.set_xlabel('$x_I$ (m)')
    ax3.set_ylabel('$y_I$ (m)')
    ax3.grid(True)
    plt.legend()
    plt.show()
