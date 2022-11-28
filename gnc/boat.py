#!/usr/bin/env python3
""" Path and trajectory planning and controls for the tethered boat
    simulation
"""
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from lqRRT import lqrrt
from path_planning import pp
from tethered_dynamics import TetheredDynamics

npl = np.linalg

path_type = 'lawnmower'  # 'lawnmower', 'line', 'trajetory' or 'data'

IS_OPEN_LOOP = False 
kp = np.array([0.2,0.2,0]) if path_type == 'trajectory' else np.repeat(0.2,2)
ki = np.array([0.00001, 0.00001, 0.0]) if path_type == 'trajectory' else np.repeat(0.00001,2)
kd = np.repeat(0, 3) if path_type == 'trajectory' else np.repeat(0.0,2)
err_accumulation = np.zeros(2)
max_ve = 4*0.44704  # mph to m/s 
saturation = np.repeat(2.0, 2)
tolerance = 0.02  # meters
err_reset_dist = 10
is_debug = False 

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
elif path_type == 'lawnmower':
    sp1=[0,0]
    sp2=[160,160]
    dx=40
    dy=40
    is_plot = False
    planning = pp(sp1,sp2,dx,dy,is_plot)
    traj = planning.trajectory(max_vel=saturation[0]) 

tension_magnitudes = [9, 11, 18, 27.5]

## Trajectory Planning
nstates = 6
ncontrols = 2
# Vehicle dimensions
boat_length = 1  # m
boat_width = 0.5  # m
goal_buffer = [6, 6, np.inf, np.inf, np.inf, np.inf]
error_tol = np.copy(goal_buffer)/2
obs = []
goal_bias = [0.7, 0.7, 0, 0, 0, 0]
goal = [25, 25, np.deg2rad(90), 0, 0, 0]

# Definition of collision
def is_feasible(x, u):
    for ob in obs:
        if npl.norm(x[:2] - ob[:2]) <= boat_length/2 + ob[2]:
            return False
    return True

def erf(xgoal, x):
    """Returns error e given two states xgoal and x.
    Angle differences are taken properly on SO3."""
    e = xgoal - x
    c = np.cos(x[2])
    s = np.sin(x[2])
    cg = np.cos(xgoal[2])
    sg = np.sin(xgoal[2])
    e[2] = np.arctan2(sg*c - cg*s, cg*c + sg*s)
    return e

def lqr(x, u):
    """Returns cost-to-go matrix S and policy matrix K given local state x and effort u.
    """
    global kp, kd
    kp_mat = np.diag(kp)
    kd_mat = np.diag(kd)
    S = np.diag([1, 1, 0.1, 0.1, 0.1, 0.1])
    K = np.hstack((kp_mat, kd_mat))
    return (S, K)

def get_boat_position(t, ii):
    """
    Return the position of the boat in a point in time
    Path types:
        'line' = empty
        'data' = get the trajectory from a csv
            for data it finds the index of the data time that most
            closely match the simulation time
    """
    if path_type == 'line' and not IS_OPEN_LOOP:
        raise ValueError('Path type "line" is only for open loop simulations')
    elif path_type == 'data':
        t_i = df_dr['time'].iloc[ii]
        while t_i <= t:
            t_i = df_dr['time'].iloc[ii]
            ii += 1
        x = df_bt['X_m'].iloc[ii]
        y = df_bt['Y_m'].iloc[ii]
        return np.array([x, y, 0]), ii
    elif path_type == 'lawnmower':
        path = 0  # TODO: implement

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

def pid_controller(t, t_last, q, q_ref , q_dot, q_ref_dot):
    """ calculate the control input (velocity) based on the desired 
        trajectory
    """
    global err_accumulation
    command = np.array([0, 0])
    dt = np.array(t - t_last)

    # Calculate error in position, orientation, and twist (velocities)
    position_error = q_ref - q 
    vel_error = q_ref_dot - q_dot 
    if np.any(kd):
        feedback_derivative = vel_error*kd[:2] 

    # Integral of error (and clip if too large)
    err_accumulation = err_accumulation + position_error*dt

    # Calculate feedback
    feedback_proportional = position_error*kp[:2] 
    feedback_integral = err_accumulation*ki[:2]
    if np.any(kd):
        feedback = feedback_proportional + feedback_derivative + feedback_integral
    else:
        feedback = feedback_proportional + feedback_integral
        
    command = np.clip(feedback, -saturation, saturation)

    if np.abs(position_error[0]) < tolerance:
        # do not move if position error is small
        print('Position error x is small: ', position_error[0]) if is_debug else None
        command[0] = 0
    if np.abs(position_error[1]) < tolerance:
        # do not move if position error is small
        print('Position error y is small: ', position_error[1]) if is_debug else None
        command[0] = 0
    elif np.abs(position_error[0]) > err_reset_dist or np.abs(position_error[1]) > err_reset_dist:
        # delete accumulated error if error is large 
        err_accumulation /= 10

    if is_debug:
        print("Feedback proportional: {}".format(
            feedback_proportional))
        if np.any(kd):
            print("Feedback derivative: {}".format(
                feedback_derivative))
        print("feedback integral: {}".format(feedback_integral))
        print("command: {}".format(command))

    command = np.array([command[0], command[1], 0])
    return command, feedback_proportional, feedback_integral    

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
        # up to 76 index is garbage data
        df_dr["time"] = df_dr["time"] - df_dr["time"].to_numpy()[76]
        df_dr = df_dr.drop(df_dr.index[range(0, 76)])
    ten_mag = tension_magnitudes[1]  # choose magnitude depending on the drones dataset 
    i_last = 0
    ii_last = 0
    t_last = 0

    # Simulation duration, timestep and animation parameters
    t0 = 0
    if path_type == 'line' or path_type == 'lawnmower' or path_type == 'trajectory':
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

    dynamics = TetheredDynamics(ten_mag=ten_mag)
    # Initial condition
    # State: [m, m, rad, m/s, m/s, rad/s]
    x0 = -14
    if path_type == 'line' or path_type == 'lawnmower' or path_type == 'trajectory':
        q0 = np.array([x0, -np.sqrt(dynamics.proj_le**2 - x0**2),
                       0, 0, 0, 0], dtype=np.float64)
        dr0 = np.array([0, 0, 0], dtype=np.float64)  # [m, m, rad]
    elif path_type == 'data':
        jj = np.argmax(df_dr['time'].to_numpy() >= t0)
        #q0 = np.array([x0 + df_dr['X_m'].iloc[jj],
        #        np.sqrt(proj_le**2 - x0**2) + df_dr['Y_m'].iloc[jj],
        #        0, 0, 0, 0], dtype=np.float64)
        q0 = np.array([df_bt['X_m'].iloc[jj], df_bt['Y_m'].iloc[jj],
                0, 0, 0, 0], dtype=np.float64)
        dr0, _ = get_drone_position(t_last, dynamics.hd, i_last)
    dynamics.dr = dr0
    dynamics.mult = 0.5
    q = np.copy(q0)

    # Define time domain
    t_arr = np.arange(t0, T, dt)

    # Preallocate results memory
    q_history = np.zeros((len(t_arr), len(q)))
    dr_history = np.zeros((len(t_arr), int(len(q)/2)))
    bt_history = np.zeros((len(t_arr), int(len(q)/2)))
    u_history = np.zeros((len(t_arr), int(len(q)/2)))
    u_world_history = np.zeros((len(t_arr), int(len(q)/2)))
    head_dr = np.zeros((len(t_arr)))
    dr_v_hist = np.zeros((len(t_arr), 2)) 
    p_cmd_hist = np.zeros((len(t_arr), 2))
    i_cmd_hist = np.zeros((len(t_arr), 2))

    if path_type == 'trajectory':
        sample_space = [(q0[0], goal[0]),
                        (q0[1], goal[1]),
                        (-np.pi, np.pi),
                        (0, saturation[0]),
                        (-saturation[1]/2, saturation[1]/2),
                        (-0.2, 0.2)]
        constraints = lqrrt.Constraints(nstates=nstates, ncontrols=ncontrols,
                                        goal_buffer=goal_buffer, is_feasible=is_feasible)
        planner = lqrrt.Planner(dynamics.step, lqr, constraints,
                                horizon=2, dt=dt, FPR=0.5,
                                error_tol=error_tol, erf=erf,
                                min_time=1, max_time=2, max_nodes=1E5,
                            goal0=goal)

        planner.update_plan(q0, sample_space, goal_bias=goal_bias, finish_on_goal=True)
    
    dynamics.dr = dr0
    print("dr0: {}".format(dr0))
    # Integrate dynamics using first-order forward stepping
    for i, t in enumerate(t_arr):

        #if df_dr.shape[0] == i:
        #    print('End of data')

        q_ref = planner.get_state(t) if path_type == 'trajectory' else None
        # Rope length constraint
        if npl.norm(q[:2] - dynamics.dr[:2]) <= dynamics.proj_le or t == 0.0:
            if IS_OPEN_LOOP:
                dynamics.dr, i_last = get_drone_position(t_last, dynamics.hd, i_last)
            else:
                if path_type == 'data':
                    # q_ref is boat position from the data
                    q_ref, ii_last = get_boat_position(t, ii_last)
                    # closed loop controller
                    q_dot = 0
                    q_ref_dot = 0
                elif path_type == 'trajectory':
                    # Planner's decision
                    u_ref = planner.get_effort(t)

                    # Controller's decision
                    kk = lqr(q, u_ref)[1]
                    ee = erf(q_ref, np.copy(q))
                    v_dr = kk.dot(ee)
                    #print("lqr: ", lqr(q, uref)[1])
                    #print("erf: ", erf(q_ref, np.copy(q)))
                    #print("v_dr: ", v_dr)
                elif path_type == 'lawnmower':
                    q_ref = traj[ii_last]
                    ee = erf(q_ref, np.copy(q))
                    if npl.norm(ee[:2]) < tolerance*2: 
                        ii_last = ii_last + 1

                v_dr, p_cmd, i_cmd = pid_controller(t, t_last, q[:2], q_ref[:2], q_dot, q_ref_dot)
            t_last += dt
        else:
            v_dr = np.array([0,0,0])

        q_history[i] = q
        dr_history[i] = dynamics.dr
        u_history[i] = dynamics.u
        u_world_history[i] = dynamics.ten
        head_dr[i] = np.degrees(np.arctan2(dynamics.diff_pos[1],dynamics.diff_pos[0]))
        if not IS_OPEN_LOOP:
            if path_type == 'data' or path_type == 'lawnmower':
                bt_history[i] = q_ref
                dr_v_hist[i] = v_dr
                p_cmd_hist[i] = p_cmd
                i_cmd_hist[i] = i_cmd
            else:
                bt_history[i] = q_ref[:3] 
                dr_v_hist[i] = v_dr[:2]

        # Step forward, qnext = qlast + qdot*dt
        q = dynamics.step(q, v_dr, dt)
    
    ## PLOTS

    # Figure for individual results
    fig1 = plt.figure()
    fig1.suptitle('State Evolution', fontsize=20)
    fig1rows = 2
    fig1cols = 5

    # Plot x position
    ax = fig1.add_subplot(fig1rows, fig1cols, 1)
    ax.set_title('X Position (m)', fontsize=16)
    ax.plot(t_arr, q_history[:, 0], 'g', label="sim boat")
    ax.plot(t_arr, dr_history[:, 0], 'b', label="drone")
    if path_type == 'data': ax.plot(t_arr, bt_history[:,0], 'r', label='boat reference (data)')
    ax.grid(True)
    ax.legend()

    # Plot y position
    ax = fig1.add_subplot(fig1rows, fig1cols, 2)
    ax.set_title('Y Position (m)', fontsize=16)
    ax.plot(t_arr, q_history[:, 1], 'g',
            t_arr, dr_history[:, 1], 'b')
    if path_type == 'data': ax.plot(t_arr, bt_history[:,1], 'r')
    ax.grid(True)

    # Plot orientation
    ax = fig1.add_subplot(fig1rows, fig1cols, 3)
    ax.set_title('Heading (deg)', fontsize=16,)
    ax.plot(t_arr, np.rad2deg(q_history[:, 2]), 'g', label="sim boat heading")
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
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True)

    # Dist drone and boat
    ax = fig1.add_subplot(fig1rows, fig1cols, 9)
    ax.set_title('Distance drone and boat (m)', fontsize=16)
    ax.plot(t_arr, dr_history[:, 0] - q_history[:, 0], 'b', label='x [m]')
    ax.plot(t_arr, dr_history[:, 1] - q_history[:, 1], 'g', label='y [m]')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True)

    # Dist norm drone and boat
    ax = fig1.add_subplot(fig1rows, fig1cols, 10)
    ax.set_title('Norm of Distance (m)', fontsize=16)
    ax.plot(t_arr, np.sqrt((q_history[:, 0] - dr_history[:, 0])**2 + (
        q_history[:, 1] - dr_history[:, 1])**2), 'k')
    ax.plot([t_arr[0], t_arr[-1]], [dynamics.proj_le, dynamics.proj_le], '-r', label='rope length')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True)
    plt.show()

    if path_type == 'data': 
        print("Mean Square Error X: ", np.mean((q_history[:, 0] - bt_history[:, 0])**2))
        print("Mean Square Error Y: ", np.mean((q_history[:, 1] - bt_history[:, 1])**2))
        fig2 = plt.figure()
        fig2.suptitle('Error Evolution', fontsize=20)
        fig1rows = 1
        fig1cols = 5  
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 1)
        ax1.set_title('X Error (m)', fontsize=16)
        ax1.plot(t_arr, bt_history[:, 0] - q_history[:, 0], 'k')
        ax1.grid(True)
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 2)
        ax1.set_title('Y Error (m)', fontsize=16)
        ax1.plot(t_arr, bt_history[:, 1] - q_history[:, 1], 'b')
        ax1.grid(True)
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 3)
        ax1.set_title('Drone vel (m/s)', fontsize=16)
        ax1.plot(t_arr, dr_v_hist[:, 0], 'k', label="vx")
        ax1.plot(t_arr, dr_v_hist[:, 1], 'b', label="vy")
        ax1.grid(True)
        ax1.legend()
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 4)
        ax1.set_title('P command (m/s)', fontsize=16)
        ax1.plot(t_arr, p_cmd_hist[:, 0], 'k', label="x")
        ax1.plot(t_arr, p_cmd_hist[:, 1], 'b', label="y")
        ax1.grid(True)
        ax1.legend()
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 5)
        ax1.set_title('I command (m/s)', fontsize=16)
        ax1.plot(t_arr, i_cmd_hist[:, 0], 'k', label="x")
        ax1.plot(t_arr, i_cmd_hist[:, 1], 'b', label="y")
        ax1.grid(True)
        ax1.legend()
        plt.show()
    elif path_type == 'trajectory':
        print("Mean Square Error X: ", np.mean((q_history[:, 0] - bt_history[:, 0])**2))
        print("Mean Square Error Y: ", np.mean((q_history[:, 1] - bt_history[:, 1])**2))
        fig2 = plt.figure()
        ax1 = fig2.add_subplot(1, 1, 1)
        dx = 0; dy = 1
        ax1.set_xlabel('- State {} +'.format(dx))
        ax1.set_ylabel('- State {} +'.format(dy))
        ax1.grid(True)
        for ID in range(planner.tree.size):
            x_seq = np.array(planner.tree.x_seq[ID])
            if ID in planner.node_seq:
                ax1.plot((x_seq[:, dx]), (x_seq[:, dy]), color='r', zorder=2)
            else:
                ax1.plot((x_seq[:, dx]), (x_seq[:, dy]), color='0.75', zorder=1)
        ax1.scatter(planner.tree.state[0, dx], planner.tree.state[0, dy], color='b', s=48, label='Start')
        ax1.scatter(planner.tree.state[planner.node_seq[-1], dx], planner.tree.state[planner.node_seq[-1], dy], color='r', s=48, label='sequence')
        ax1.scatter(goal[dx], goal[dy], color='g', s=48, label='Goal')
        ax1.legend()
        for ob in obs:
            ax1.add_patch(plt.Circle((ob[0], ob[1]), radius=ob[2], fc='r'))


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
    rope = np.array([dr_history[0, 0] - q_history[0, 0],
                    dr_history[0, 1] - q_history[0, 1],])
    rope = rope* dynamics.proj_le / np.linalg.norm(rope) 
    p = ax6.scatter(q_history[0, 0], q_history[0, 1], color='k', s=pthick, label='simulated boat')
    h = ax6.plot([q_history[0, 0], q_history[0, 0] + llen*np.cos(q_history[0, 2])],
                 [q_history[0, 1], q_history[0, 1] + llen*np.sin(q_history[0, 2])], color='k', linewidth=lthick)
    pref = ax6.scatter(dr_history[0, 0], dr_history[0, 1], color='b', s=pthick, label='drone')
    href = ax6.plot([dr_history[0, 0], dr_history[0, 0] + rope[0]], \
                    [dr_history[0, 1], dr_history[0, 1] + rope[1]], color='r', linewidth=lthick)
    pact = ax6.scatter(bt_history[0, 0], bt_history[0, 1], color='c', s=pthick, label='boat data')

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
    plt.legend()
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
