#!/usr/bin/env python3
""" Path and trajectory planning and controls for the tethered boat
    simulation

    User can choose between different control types with IS_OPEN_LOOP. Open loop,
    means the drone position is already determined. Closed loop means we need to
    calculate the drone position based on the control input. The user also chooses
    the path_type:
        'line' = straight line drone and boat path if open loop
        'lawnmower' = lawnmower pattern for the boat reference. Also for the drone if open loop
        'obstacle' = single goal waypoint with obstacle in the middle. Use lqRRT for drone so only closed loop
        'trajectory' = use lawnmower for boat reference and lqRRT for drone so only closed loop
        'data' = trajectory from data collected in the field, saved as a csv, and used as the boat
        reference. Also drone if open loop
"""
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from lqRRT import lqrrt
from open_loop_dynamics import OpenLoopDynamics
from path_planning import pp
from tethered_dynamics import TetheredDynamics
from stanley_controller import StanleyController
from integral_stanley import IntegralStanley
from waypoint_gen import interpolate_points, plot_points

npl = np.linalg

IS_OPEN_LOOP = True

path_type = "raster"  # 'lawnmower', 'line', 'obstacle', 'trajectory', 'raster' or 'data'
# don't use "trajectory or obstacle path type, as they use lqrrt planner"
raster = [[1,1], [1,30], [5,30], [5,1], [10,1], [10,30], [15,30], [15,1], [20,1], [20,30], [25,30], [25,1], [30,1], [30,30], [35,30], [35,1], [40,1], [40,30], [45,30], [45,1], [50,1], [50,30], [55,30], [55,1], [60,1], [60,30], [65,30], [65,1], [70,1], [70,30], [75,30], [75,1], [80,1], [80,30], [85,30], [85,1], [90,1], [90,30], [95,30], [95,1], [100,1], [100,30]]
raster_v = 1  # m/s
raster_pt_num = 10000

# Controller parameters
kp = np.array([0.08, 0.08, 0])
ki = np.array([0.05, 0.05, 0.0])
kd = np.array([0.0, 0.0, 0.0])
err_accumulation = np.zeros(2)
# max_ve = 4*0.44704  # mph to m/s
saturation = np.repeat(1.8, 2)  # max vel of drone [m/s]
tolerance = 0.2  # meters
traj_tolerance = 15.0
err_reset_dist = 50
is_debug = False

# Rudder Control parameters 
rudder_control = "step"  # 'none', 'step', 'stanley', 'Integral_stanley', or 'MPC'
alpha_r = 0.6  # Angular gain for Stanley Controller
k_r = 0.5  # Control gain for Stanley Controller 

#extra parameter for integral_stanley controller
kp_r = 0.2  # Integral gain for integral stanley controller 

if path_type == "data":
    # Get data collected in the field
    traj_path = "csv/traj_data/"   
    force_files = ["force_traj_3", "force_traj_6", "force_traj_9", "force_traj_12"]
    drone_files = ["drone_traj_3", "drone_traj_6", "drone_traj_9", "drone_traj_12"]
elif path_type == "lawnmower" or path_type == "trajectory":
    sp1 = [0, 0]
    sp2 = [120, 80]
    dy = 60
    is_plot = False
    planning = pp(sp1, sp2, dy, is_plot)
    traj = planning.trajectory(max_vel=saturation[0])
    traj = traj[: traj.shape[0] // 2, :]  # short data

tension_magnitudes = [9, 11, 18, 27.5]

## Trajectory Planning
nstates = 9
ncontrols = 3
# Vehicle dimensions
boat_length = 1  # m
boat_width = 0.5  # m
goal_buffer = [2, 2, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]
error_tol = np.copy(goal_buffer) / 2
if path_type == "obstacle":
    obs = np.array(
        [
            [3, -2, 2],
        ]
    )
    # Goals for the drone position ignored
    goal = [10, 10, np.deg2rad(90), 0, 0, 0, 0, 0, 0]
elif path_type == "trajectory":
    obs = []
goal_bias = [0.3, 0.3, 0, 0, 0, 0, 0, 0, 0]


def gen_ss(seed, goal, buff=[1] * 4):
    """
    Returns a sample space given a seed state, goal state, and buffer.
    The drone position is not sampled because it is determined by the
    control input.
    """
    return [
        (min([seed[0], goal[0]]) - buff[0], max([seed[0], goal[0]]) + buff[1]),
        (min([seed[1], goal[1]]) - buff[2], max([seed[1], goal[1]]) + buff[3]),
        (-np.pi, np.pi),
        (-0.0, 2),
        (-0.2, 0.2),
        (-0.8, 0.8),
        (seed[6], seed[6]),
        (seed[7], seed[7]),
        (seed[8], seed[8]),
    ]


# Definition of collision
def is_feasible(x, u):
    for ob in obs:
        if npl.norm(x[:2] - ob[:2]) <= boat_length / 2 + ob[2]:
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
    e[2] = np.arctan2(sg * c - cg * s, cg * c + sg * s)
    return e


def lqr(x, u):
    """Returns cost-to-go matrix S and policy matrix K given local state x and effort u."""
    global kp, kd
    kp_mat = np.diag(kp)
    kd_mat = np.diag(kd)
    S = np.diag([1, 1, 0.1, 0.0, 0.0, 0.1, 0, 0, 0])
    K = np.hstack((kp_mat, kd_mat, np.zeros((3, 3))))
    return (S, K)


## Get data from dataframe
def get_boat_position(t, ii):
    """
    Return the position of the boat in a point in time
    Path types:
        'line' = straight line
        'data' = get the trajectory from a csv
            for data it finds the index of the data time that most
            closely match the simulation time
    """
    if path_type == "line":
        mx = 0.7
        my = 0.7
        return np.array([mx * t, my * t, np.arctan2(my, mx), mx, my, 0]), 0
    else:
        t_i = df_dr["time"].iloc[ii]
        while t_i <= t:
            t_i = df_dr["time"].iloc[ii]
            ii += 1
        x = df_bt["X_m"].iloc[ii]
        y = df_bt["Y_m"].iloc[ii]
        return np.array([x, y, 0, 0, 0, 0]), ii
    
def get_boat_reference():  # get the reference trajectory of the boat, but in this case uses drone reference for controller because it is smoother 
    '''Return the reference trajectory of the boat'''
    ref = []
    for ii in range(len(df_bt["X_m"].iloc[:])):
        x = df_bt["X_m"].iloc[ii]
        y = df_bt["Y_m"].iloc[ii]
        ref.append([x,y])
    return np.array(ref)

def get_drone_position(t, hd, ii):
    """
    Return the position of the drone in a point in time,
    depending on the desired velocity
        = get the trajectory from a csv
            for data it finds the index of the data time that most
            closely match the simulation time
    """
    t_i = df_dr["time"].iloc[ii]
    while t_i <= t:
        t_i = df_dr["time"].iloc[ii]
        ii += 1
    x = df_dr["X_m"].iloc[ii]
    y = df_dr["Y_m"].iloc[ii]
    return np.array([x, y, hd]), ii

def precompute_distances(ref):
    '''Precompute the distances between the points in the reference trajectory to save computation time'''
    distances = np.zeros(len(ref))
    for i in range(1, len(ref)):
        distances[i] = distances[i - 1] + np.abs(ref[i][0] - ref[i - 1][0]) + np.abs(ref[i][1] - ref[i - 1][1])
    return distances

def get_raster_position(t, raster_v, ref, distances):
    '''Return the position of the drone at a point in time, depending on the desired velocity'''
    d = t * raster_v
    idx = np.searchsorted(distances, d, side='right')
    if idx == 0:
        return np.array([ref[0][0], ref[0][1], raster_v]), 0
    else:
        d_c = distances[idx - 1]
        position = np.array([ref[idx - 1][0], ref[idx - 1][1], raster_v])
        return position, idx - 1 if d - d_c < d - distances[idx] else idx


def pid_controller(t, t_last, q, q_ref, q_dot, q_ref_dot, err_accumulation):
    """calculate the control input (velocity) based on the desired
    trajectory
    """
    command = np.array([0, 0])
    dt = np.array(t - t_last)

    # Calculate error in position, orientation, and twist (velocities)
    position_error = q_ref - q
    vel_error = q_ref_dot - q_dot
    if np.any(kd):
        feedback_derivative = vel_error[:2] * kd[:2]

    # Integral of error (and clip if too large)
    err_accumulation = err_accumulation + position_error * dt

    # Calculate feedback
    feedback_proportional = position_error * kp[:2]
    feedback_integral = err_accumulation * ki[:2]
    if np.any(kd):
        feedback = feedback_proportional + feedback_derivative + feedback_integral
    else:
        feedback = feedback_proportional + feedback_integral

    command = np.clip(feedback, -saturation, saturation)

    if np.abs(position_error[0]) < tolerance:
        # do not move if position error is small
        print("Position error x is small: ", position_error[0]) if is_debug else None
        command[0] = 0
    if np.abs(position_error[1]) < tolerance:
        # do not move if position error is small
        print("Position error y is small: ", position_error[1]) if is_debug else None
        command[0] = 0
    elif (
        np.abs(position_error[0]) > err_reset_dist
        or np.abs(position_error[1]) > err_reset_dist
    ):
        # delete accumulated error if error is large
        err_accumulation /= 2

    if is_debug:
        print(f"Feedback proportional: {feedback_proportional}")
        if np.any(kd):
            print(f"Feedback derivative: {feedback_derivative}")
        print(f"feedback integral: {feedback_integral}")
        print(f"command: {command}")

    command = np.array([command[0], command[1], 0])
    return command, feedback_proportional, feedback_integral, err_accumulation




def get_R(q):
    """Rotation matrix (orientation, converts body to world)"""
    return np.array(
        [[np.cos(q[2]), -np.sin(q[2]), 0], [np.sin(q[2]), np.cos(q[2]), 0], [0, 0, 1]]
    )


# SIMULATION
if __name__ == "__main__":

    # Choose one option from the datasets
    data_file_index = 1
    if path_type == "data":
        # Choose one option from the datasets (drones and echos)
        df_dr = pd.read_csv(traj_path + drone_files[data_file_index] + ".csv")
        df_bt = pd.read_csv(traj_path + force_files[data_file_index] + ".csv")

        # Fix delta in time
        # kk = np.argmax(df_dr['Y_m'].to_numpy() <= df_bt['Y_m'].to_numpy()[0])
        # tt = df_dr["time"].to_numpy()[80]
        # up to 76 index is garbage data
        df_dr["time"] = df_dr["time"] - df_dr["time"].to_numpy()[76]
        df_dr = df_dr.drop(df_dr.index[range(0, 76)])
    elif path_type == 'raster':
        ref = np.array(interpolate_points(raster, raster_pt_num))
        distances = precompute_distances(ref)

    # choose magnitude depending on the drones dataset
    ten_mag = tension_magnitudes[data_file_index]
    i_last = 0
    ii_last = 0
    t_last = 0

    if IS_OPEN_LOOP:
        dynamics = OpenLoopDynamics(ten_mag=ten_mag)
    else:
        dynamics = TetheredDynamics(ten_mag=ten_mag)
    # Initial condition
    # State: [m, m, rad, m/s, m/s, rad/s]
    x0 = -5
    if (
        path_type == "line"
        or path_type == "lawnmower"
        or path_type == "obstacle"
        or path_type == "trajectory"
    ):
        y0 = 0 if path_type == "obstacle" else 30
        dr0 = np.array([2, y0, 0], dtype=np.float64)  # [m, m, rad]
        q0 = np.array(
            [
                dr0[0] + x0,
                dr0[1] - np.sqrt(dynamics.proj_le**2 - x0**2),
                0,
                0,
                0,
                0,
            ],
            dtype=np.float64,
        )  # boat state [m, m, rad, m/s, m/s, rad/s]
        goal_counter = 0
        index_counter = 0
        is_same_goal = False
    elif path_type == "data":
        jj = np.argmax(df_dr["time"].to_numpy() >= 0)
        # q0 = np.array([x0 + df_dr['X_m'].iloc[jj],
        #        np.sqrt(proj_le**2 - x0**2) + df_dr['Y_m'].iloc[jj],
        #        0, 0, 0, 0], dtype=np.float64)
        q0 = np.array(
            [df_bt["X_m"].iloc[jj], df_bt["Y_m"].iloc[jj], 0, 0, 0, 0], dtype=np.float64
        )
        dr0, _ = get_drone_position(t_last, dynamics.hd, i_last)
        dynamics.dr = np.copy(dr0)
    elif path_type == "raster":
        q0 = np.array([ref[0][0], ref[0][1], 0, 0, 0, 0], dtype=np.float64)
        dr0, _ = get_raster_position(t_last, raster_v, ref, distances)
        dynamics.dr = np.copy(dr0)

    dr = np.copy(dr0)
    q = np.copy(q0)
    x = np.concatenate((q0, dr0))

    ## Setup lqRRT planner
    if path_type == "obstacle" or path_type == "trajectory":
        constraints = lqrrt.Constraints(
            nstates=nstates,
            ncontrols=ncontrols,
            goal_buffer=goal_buffer,
            is_feasible=is_feasible,
        )
        if path_type == "obstacle":
            goal0 = goal
        else:
            goal0 = None
            T_last = 0

        planner = lqrrt.Planner(
            dynamics.step,
            lqr,
            constraints,
            horizon=2,
            dt=0.01,
            FPR=0.5,
            error_tol=error_tol,
            erf=erf,
            min_time=1,
            max_time=20,
            max_nodes=1e5,
            goal0=goal0,
        )
        t_delta = 5
        real_tol = [
            2,
            2,
            np.deg2rad(20),
            np.inf,
            np.inf,
            np.inf,
            np.inf,
            np.inf,
            np.inf,
        ]

        if path_type == "obstacle":
            sample_space = gen_ss(x, goal0, [1, 1, 1, 1])
            planner.update_plan(
                np.copy(x), sample_space, goal_bias=goal_bias, finish_on_goal=True
            )
        else:  # path_type == "trajectory":
            s0 = np.copy(x)
            get_state_history = []
            u_ref_history = []
            Ts = np.zeros(traj.shape[0])
            for i in range(traj.shape[0]):
                # go to new waypoint, generate and plan the path
                planner.set_goal(traj[i, :])
                sample_space = gen_ss(s0, traj[i, :], [1, 1, 1, 1])
                planner.update_plan(
                    s0, sample_space, goal_bias=goal_bias, finish_on_goal=True
                )
                # get the results  and save them to later use them in order
                Ts[i] = planner.T
                states = np.zeros((int(planner.T / 0.01) + 1, 9))
                efforts = np.zeros((int(planner.T / 0.01) + 1, 3))
                for ii, t in enumerate(np.arange(0, planner.T, 0.01)):
                    states[ii] = planner.get_state(t)
                    efforts[ii] = planner.get_effort(t)
                get_state_history.append(states)
                u_ref_history.append(efforts)
                # next initial state
                s0 = planner.get_state(planner.T)
                print("Goal: ", traj[i, :])
                print("Goal number: ", i)

    print("=" * 50)
    print("Starting simulation")
    print(f"dr0: {dr0}")
    print(f"q0: {q0}")
    # Simulation duration, timestep and animation parameters
    t0 = 0
    if path_type == "line":
        T = 150  # s
        dt = 0.001
    if path_type == "raster":
        T = 500
        dt = 0.01
    elif path_type == "lawnmower":
        dist_between_pts = 7
        T = (
            traj.shape[0] * dist_between_pts / 2
        )  # s (decrease dist_between_pts if T is too large)
        dt = 0.01
    elif path_type == "data":
        T = np.min([df_bt["time1"].to_numpy()[-1], df_dr["time"].to_numpy()[-1]])
        dt = 0.01
    elif path_type == "obstacle":
        T = planner.T
        dt = 0.01
    elif path_type == "trajectory":
        dt = 0.01
        T = Ts.sum()

    #dt = 1  #make time to be every 1 second
    print("T: ", T)

    # Define time domain
    t_arr = np.arange(t0, T, dt)

    #set up controller for rudder 
    if path_type == 'data':
        if rudder_control == "stanley":
            ref = get_boat_reference()
            ref = ref[:int(len(ref)/2)]
            controller = StanleyController(angular_gain=alpha_r, control_gain = k_r, reference=ref, debug=is_debug)
        elif rudder_control == "Integral_stanley":
            ref = get_boat_reference()
            ref = ref[:int(len(ref)/2)]
            controller = IntegralStanley(angular_gain=alpha_r, control_gain = k_r, Integral_gain = kp_r, reference=ref, debug=is_debug)
    elif path_type == 'raster':
        if rudder_control == "stanley":
            controller = StanleyController(angular_gain=alpha_r, control_gain = k_r, reference=ref, debug=is_debug)
        elif rudder_control == "Integral_stanley":
            controller = IntegralStanley(angular_gain=alpha_r, control_gain = k_r, Integral_gain = kp_r, reference=ref, debug=is_debug)

    # Preallocate results memory
    q_history = np.zeros((len(t_arr), 6))
    bt_history = np.zeros((len(t_arr), 6))
    dr_history = np.zeros((len(t_arr), 3))
    u_history = np.zeros((len(t_arr), 3))
    u_rudder_history = np.zeros((len(t_arr), 3))
    u_world_history = np.zeros((len(t_arr), 3))
    head_dr = np.zeros(len(t_arr))
    dr_v_hist = np.zeros((len(t_arr), ncontrols))
    p_cmd_hist = np.zeros((len(t_arr), 2))
    i_cmd_hist = np.zeros((len(t_arr), 2))
    delta_history = np.zeros(len(t_arr))
    crosstrack_error_history = np.zeros(len(t_arr))
    steering_error_history = np.zeros(len(t_arr))
    heading_error_history = np.zeros(len(t_arr))
    d_history = np.zeros(len(t_arr))
    dx_history = np.zeros(len(t_arr))
    dy_history = np.zeros(len(t_arr))
    current_heading_history = np.zeros(len(t_arr))
    reference_heading_history = np.zeros(len(t_arr))
    current_velocity_history = np.zeros(len(t_arr))

    # Integrate dynamics using first-order forward stepping
    for i, t in enumerate(t_arr):

        q_dot = q[3:6]
        if path_type == "lawnmower":
            if traj.shape[0] == ii_last:
                print("End of data. Time=", t)
                break
            q_ref = traj[ii_last]
            q_ref_dot = traj[ii_last][3:6]
            ee = erf(q_ref[:3], np.copy(q[:3]))
            if npl.norm(ee[:2]) < traj_tolerance:
                ii_last = ii_last + 1
        elif path_type == "data" or path_type == "line":
            # q_ref is boat position from the data
            q_ref, ii_last = get_boat_position(t, ii_last)
            q_ref_dot = 0
        elif path_type == "raster":
            q_ref, ii_last = get_raster_position(t, raster_v, ref, distances)
            q_ref = np.append(q_ref, [0, 0, 0])
            q_ref_dot = 0

        if IS_OPEN_LOOP:
            if npl.norm(q[:2] - dr[:2]) <= dynamics.proj_le + dynamics.dL:
                if path_type == "data":
                    dynamics.dr, i_last = get_drone_position(
                        t_last, dynamics.hd, i_last
                    )
                elif path_type == "raster":
                    dynamics.dr, i_last = get_raster_position(
                        t, raster_v, ref, distances
                    )
                elif path_type == "lawnmower":
                    dynamics.dr = q_ref[:3]
                t_last += dt
            if path_type == "line":
                q_ref = np.concatenate((dr, np.zeros(3)))
            v_dr = np.array([0, 0, 0])
            dr = dynamics.dr
        else:
            if path_type == "obstacle":
                # Planner's decision
                u_ref = planner.get_effort(t)
                q_ref = planner.get_state(t)
                q_ref_dot = q_ref[3:6]
            elif path_type == "trajectory":
                # TIME ATTEMPT
                # if t > t_delta:
                #    planner.set_goal(traj[goal_counter, :])
                #    planner.update_plan(
                #        q, sample_space, goal_bias=goal_bias, finish_on_goal=True
                #    )
                #    t_delta += t_delta
                # err_now = np.abs(erf(traj[goal_counter, :], q))
                # if np.all(err_now <= real_tol):
                #    goal_counter += 1
                # q_ref = planner.get_state(t)
                # q_ref_dot = q_ref[3:]

                if t >= T_last + Ts[goal_counter]:
                    T_last += Ts[goal_counter]
                    goal_counter += 1
                    index_counter = 0
                    if goal_counter >= traj.shape[0]:
                        print("End of data. Time=", t)
                        break

                u_ref = u_ref_history[goal_counter][index_counter]
                q_ref = get_state_history[goal_counter][index_counter]
                # u_ref = u_ref_history[goal_counter].get_effort(t - T_last)
                # q_ref = plan_history[goal_counter].get_state(t - T_last)
                q_ref_dot = q_ref[3:6]
                index_counter += 1

            v_dr, p_cmd, i_cmd, err_accumulation = pid_controller(
                t, t_last, q[:2], q_ref[:2], q_dot, q_ref_dot, err_accumulation
            )
            v_dr = np.copy(u_ref) if path_type == "obstacle" else v_dr
            t_last += dt

        bt_history[i] = q_ref[:6]
        q_history[i] = q
        dr_history[i] = dr
        u_history[i] = dynamics.u
        u_rudder_history[i] = dynamics.u_rudder
        u_world_history[i] = dynamics.ten
        head_dr[i] = np.degrees(np.arctan2(dynamics.diff_pos[1], dynamics.diff_pos[0]))
        if not IS_OPEN_LOOP:
            dr_v_hist[i] = v_dr
            p_cmd_hist[i] = p_cmd
            i_cmd_hist[i] = i_cmd

        if rudder_control == "none":
            delta = 0
        elif rudder_control == "step":
            if (t / 20) % 2 < 1:  # change the rudder control angle every 20 seconds
                delta = 24
            else:
                delta = -24
        elif rudder_control == "stanley" or rudder_control == "Integral_stanley":  # controller setup was done before the loop
            delta, crosstrack_error, steering_error, heading_error = controller.stanley_control(q[0],q[1],dt)

            delta_history[i] = delta
            crosstrack_error_history[i] = crosstrack_error
            steering_error_history[i] = steering_error
            heading_error_history[i] = heading_error

            d_history[i],dx_history[i],dy_history[i], current_heading_history[i], reference_heading_history[i], current_velocity_history[i] = controller.print_debug(q[0],q[1],dt)
            controller.update_position(q[0],q[1])

        # Step forward, x_next = x_last + x_dot*dt
        x = np.concatenate((q, dr)) if not IS_OPEN_LOOP else q
        x = dynamics.step(x, v_dr, dt, delta)
        q = x[:6]
        dr = x[6:] if not IS_OPEN_LOOP else dr

    ## PLOTS
    print("\nPlotting...")

    # Figure for individual results
    fig1 = plt.figure()
    fig1.suptitle("State Evolution", fontsize=20)
    fig1rows = 2
    fig1cols = 5

    # Plot x position
    ax = fig1.add_subplot(fig1rows, fig1cols, 1)
    ax.set_title("X Position (m)", fontsize=16)
    ax.plot(t_arr, q_history[:, 0], "-g", label="sim boat")
    ax.plot(t_arr, dr_history[:, 0], "--b", label="drone")
    if not IS_OPEN_LOOP:
        ax.plot(t_arr, bt_history[:, 0], "-.r", label="boat reference boat(data)")
        ax.plot(t_arr, bt_history[:, 0] - traj_tolerance, "--r")
        ax.plot(t_arr, bt_history[:, 0] + traj_tolerance, "--r")
    ax.grid(True)
    ax.legend()
    # Plot y position
    ax = fig1.add_subplot(fig1rows, fig1cols, 2)
    ax.set_title("Y Position (m)", fontsize=16)
    ax.plot(t_arr, q_history[:, 1], "-g", t_arr, dr_history[:, 1], "--b")
    if not IS_OPEN_LOOP:
        ax.plot(t_arr, bt_history[:, 1], "-.r")
        ax.plot(t_arr, bt_history[:, 1] - traj_tolerance, "--r")
        ax.plot(t_arr, bt_history[:, 1] + traj_tolerance, "--r")
    ax.grid(True)

    # Plot orientation
    ax = fig1.add_subplot(fig1rows, fig1cols, 3)
    ax.set_title(
        "Heading (deg)",
        fontsize=16,
    )
    ax.plot(t_arr, np.rad2deg(q_history[:, 2]), "g", label="sim boat heading")
    ax.plot(t_arr, head_dr, "k", label="heading from boat to drone")
    ax.grid(True)
    ax.legend()

    # Plot control efforts
    ax = fig1.add_subplot(fig1rows, fig1cols, 4)
    ax.set_title("Body-fixed Wrench (N, N, N*m)", fontsize=16)
    ax.plot(t_arr, u_history[:, 0], "b", label="x [N]")
    ax.plot(t_arr, u_history[:, 1], "g", label="y [N]")
    ax.plot(t_arr, u_history[:, 2], "r", label="z [N m]")
    ax.plot(t_arr[0:-1:200], u_rudder_history[0:-1:200, 0], "--b", label="x rudder [N]")
    ax.plot(t_arr[0:-1:200], u_rudder_history[0:-1:200, 1], "--g", label="y rudder [N]")
    ax.plot(
        t_arr[0:-1:200], u_rudder_history[0:-1:200, 2], "--r", label="z rudder [N m]"
    )
    ax.legend()
    ax.grid(True)

    # Plot x velocity
    ax = fig1.add_subplot(fig1rows, fig1cols, 6)
    ax.set_title("Surge (m/s)", fontsize=16)
    ax.plot(
        t_arr,
        q_history[:, 3],
        "g",
    )
    ax.set_xlabel("Time (s)")
    ax.grid(True)

    # Plot y velocity
    ax = fig1.add_subplot(fig1rows, fig1cols, 7)
    ax.set_title("Sway (m/s)", fontsize=16)
    ax.plot(
        t_arr[0:-1:200],
        q_history[0:-1:200, 4],
        "g",
    )
    ax.set_xlabel("Time (s)")
    ax.grid(True)

    # Plot yaw velocity
    ax = fig1.add_subplot(fig1rows, fig1cols, 8)
    ax.set_title("Yaw (deg/s)", fontsize=16)
    ax.plot(
        t_arr[0:-1:200],
        np.rad2deg(q_history[0:-1:200, 5]),
        "g",
    )
    ax.set_xlabel("Time (s)")
    ax.grid(True)

    # Wrench in world frame
    ax = fig1.add_subplot(fig1rows, fig1cols, 5)
    ax.set_title("World Wrench for Tension [N, N, Nm]", fontsize=16)
    ax.plot(t_arr, u_world_history[:, 0], "b", label="x [N]")
    ax.plot(t_arr, u_world_history[:, 1], "g", label="y [N]")
    ax.plot(t_arr, u_world_history[:, 2], "r", label="z [N m]")
    ax.set_xlabel("Time (s)")
    ax.legend()
    ax.grid(True)

    # Dist drone and boat
    ax = fig1.add_subplot(fig1rows, fig1cols, 9)
    ax.set_title("Distance drone and boat (m)", fontsize=16)
    ax.plot(t_arr, dr_history[:, 0] - q_history[:, 0], "b", label="x [m]")
    ax.plot(t_arr, dr_history[:, 1] - q_history[:, 1], "g", label="y [m]")
    ax.set_xlabel("Time (s)")
    ax.legend()
    ax.grid(True)

    # Dist norm drone and boat
    ax = fig1.add_subplot(fig1rows, fig1cols, 10)
    ax.set_title("Norm of Distance (m)", fontsize=16)
    ax.plot(
        t_arr,
        np.sqrt(
            (q_history[:, 0] - dr_history[:, 0]) ** 2
            + (q_history[:, 1] - dr_history[:, 1]) ** 2
        ),
        "k",
    )
    ax.plot(
        [t_arr[0], t_arr[-1]],
        [dynamics.proj_le, dynamics.proj_le],
        "-r",
        label="rope length",
    )
    ax.plot(
        [t_arr[0], t_arr[-1]],
        [dynamics.proj_le - dynamics.dL, dynamics.proj_le - dynamics.dL],
        "--g",
        label="force threshold",
    )
    ax.plot(
        [t_arr[0], t_arr[-1]],
        [dynamics.proj_le + dynamics.dL, dynamics.proj_le + dynamics.dL],
        "-g",
        label="no v_dr threshold",
    )
    ax.set_xlabel("Time (s)")
    ax.legend()
    ax.grid(True)
    plt.show()

    if rudder_control == "stanley" or rudder_control == "Integral_stanley":
        # plot rudder control input
        # Figure for individual results
        fig4 = plt.figure()
        fig4.suptitle("Rudder Control", fontsize=20)
        fig4rows = 3
        fig4cols = 3
        # Plot rudder angle 
        ax = fig4.add_subplot(fig4rows, fig4cols, 1)
        ax.set_title("Rudder Angle (Degree)", fontsize=16)
        ax.plot(t_arr, delta_history, "-g", label="rudder angle")
        #ax.plot(t_arr, dr_history[:, 0], "--b", label="drone")
        ax.grid(True)
        ax.legend()
        # Plot crosstrack error
        ax = fig4.add_subplot(fig4rows, fig4cols, 2)
        ax.set_title("Crosstrack Error (m)", fontsize=16)
        ax.plot(t_arr, crosstrack_error_history, "-g", label="crosstrack error")
        ax.grid(True)
        ax.legend()
        # Plot steering error
        ax = fig4.add_subplot(fig4rows, fig4cols, 3)
        ax.set_title("Steering Error (Radian)", fontsize=16)
        ax.plot(t_arr, steering_error_history, "-g", label="steering error")
        ax.grid(True)
        ax.legend()
        # Plot heading error
        ax = fig4.add_subplot(fig4rows, fig4cols, 4)
        ax.set_title("Heading Error (Radian)", fontsize=16)
        ax.plot(t_arr, heading_error_history, "-g", label="heading error")
        ax.grid(True)
        # Plot distance to the reference
        ax = fig4.add_subplot(fig4rows, fig4cols, 5)
        ax.set_title("Distance to the reference (m)", fontsize=16)
        ax.plot(t_arr, d_history, "-g", label="distance")
        ax.grid(True)
        # Plot dx to the reference
        ax = fig4.add_subplot(fig4rows, fig4cols, 6)
        ax.set_title("dx to the reference (m)", fontsize=16)
        ax.plot(t_arr, dx_history, "-g", label="dx")
        ax.grid(True)
        #plot current velocity
        ax = fig4.add_subplot(fig4rows, fig4cols, 7)
        ax.set_title("Current velocity (m/s)", fontsize=16)
        ax.plot(t_arr, current_velocity_history, "-g", label="current velocity")
        ax.grid(True)
        # Plot current heading
        ax = fig4.add_subplot(fig4rows, fig4cols, 8)
        ax.set_title("Current heading (Radian)", fontsize=16)
        ax.plot(t_arr, current_heading_history, "-g", label="current heading")
        ax.grid(True)
        # Plot reference heading
        ax = fig4.add_subplot(fig4rows, fig4cols, 9)
        ax.set_title("Reference heading (Radian)", fontsize=16)
        ax.plot(t_arr, reference_heading_history, "-g", label="reference heading")
        ax.grid(True)
        plt.show()

    if not IS_OPEN_LOOP:
        print(
            "Mean Square Error X: ", np.mean((q_history[:, 0] - bt_history[:, 0]) ** 2)
        )
        print(
            "Mean Square Error Y: ", np.mean((q_history[:, 1] - bt_history[:, 1]) ** 2)
        )
        fig2 = plt.figure()
        fig2.suptitle("Error Evolution", fontsize=20)
        fig1rows = 1
        fig1cols = 5
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 1)
        ax1.set_title("X Error (m)", fontsize=16)
        ax1.plot(t_arr, bt_history[:, 0] - q_history[:, 0], "k")
        ax1.grid(True)
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 2)
        ax1.set_title("Y Error (m)", fontsize=16)
        ax1.plot(t_arr, bt_history[:, 1] - q_history[:, 1], "b")
        ax1.grid(True)
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 3)
        ax1.set_title("Drone vel (m/s)", fontsize=16)
        ax1.plot(t_arr, dr_v_hist[:, 0], "k", label="vx")
        ax1.plot(t_arr, dr_v_hist[:, 1], "b", label="vy")
        ax1.grid(True)
        ax1.legend()
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 4)
        ax1.set_title("P command (m/s)", fontsize=16)
        ax1.plot(t_arr, p_cmd_hist[:, 0], "k", label="x")
        ax1.plot(t_arr, p_cmd_hist[:, 1], "b", label="y")
        ax1.grid(True)
        ax1.legend()
        ax1 = fig2.add_subplot(fig1rows, fig1cols, 5)
        ax1.set_title("I command (m/s)", fontsize=16)
        ax1.plot(t_arr, i_cmd_hist[:, 0], "k", label="x")
        ax1.plot(t_arr, i_cmd_hist[:, 1], "b", label="y")
        ax1.grid(True)
        ax1.legend()
        plt.show()
    if path_type == "obstacle":
        fig2 = plt.figure()
        ax1 = fig2.add_subplot(1, 1, 1)
        dx = 0
        dy = 1
        ax1.set_xlabel(f"- State {dx} +")
        ax1.set_ylabel(f"- State {dy} +")
        ax1.grid(True)
        for ID in range(planner.tree.size):
            x_seq = np.array(planner.tree.x_seq[ID])
            if ID in planner.node_seq:
                ax1.plot((x_seq[:, dx]), (x_seq[:, dy]), color="r", zorder=2)
            else:
                ax1.plot((x_seq[:, dx]), (x_seq[:, dy]), color="0.75", zorder=1)
        ax1.scatter(
            planner.tree.state[0, dx],
            planner.tree.state[0, dy],
            color="b",
            s=48,
            label="Start",
        )
        ax1.scatter(
            planner.tree.state[planner.node_seq[-1], dx],
            planner.tree.state[planner.node_seq[-1], dy],
            color="c",
            s=48,
            label="sequence",
        )
        ax1.scatter(planner.goal[dx], planner.goal[dy], color="g", s=48, label="Goal")
        ax1.legend()
        for ob in obs:
            ax1.add_patch(
                plt.Circle((ob[0], ob[1]), radius=ob[2], fc="r", label="Obstacle")
            )

    # Figure for animation
    fig6 = plt.figure()
    fig6.suptitle("Evolution")
    ax6 = fig6.add_subplot(1, 1, 1)
    ax6.set_xlabel("X (m)")
    ax6.set_ylabel("Y (m)")
    ax6.set_aspect("equal", "datalim")
    ax6.grid(True)

    # Points and lines for representing positions and headings
    pthick = 100
    lthick = 2
    llen = 2.5
    rope = np.array(
        [
            dr_history[:, 0] - q_history[:, 0],
            dr_history[:, 1] - q_history[:, 1],
        ]
    )
    rope = rope * dynamics.proj_le / np.linalg.norm(rope)
    p = ax6.scatter(
        q_history[0, 0], q_history[0, 1], color="k", s=pthick, label="simulated boat"
    )
    h = ax6.plot(
        [q_history[0, 0], q_history[0, 0] + llen * np.cos(q_history[0, 2])],
        [q_history[0, 1], q_history[0, 1] + llen * np.sin(q_history[0, 2])],
        color="k",
        linewidth=lthick,
    )
    pref = ax6.scatter(
        dr_history[0, 0],
        dr_history[0, 1],
        color="b",
        s=pthick,
        marker="s",
        label="drone",
    )
    href = ax6.plot(
        [dr_history[0, 0], dr_history[0, 0] + rope[0, 0]],
        [dr_history[0, 1], dr_history[0, 1] + rope[0, 1]],
        color="r",
        linewidth=lthick,
    )
    pact = ax6.scatter(
        bt_history[0, 0], bt_history[0, 1], color="c", s=pthick, label="boat data"
    )

    # Matplotlib Animation Parameters
    framerate = 20  # fps
    speedup = 100  # kinda makes the playback a little faster
    store_data = False  # should data be stored into a .mat?
    outline_path = True  # show path outline on animation?

    # Plot entirety of actual obstacle
    if outline_path:
        ax6.plot(
            q_history[:, 0],
            q_history[:, 1],
            "k--",
            dr_history[:, 0],
            dr_history[:, 1],
            "g--",
            bt_history[:, 0],
            bt_history[:, 1],
            "m--",
        )

    # Function for updating the animation frame

    def update_ani(arg, ii=[0]):

        i = ii[0]  # don't ask...

        if np.isclose(t_arr[i], np.around(t_arr[i], 1)):
            fig6.suptitle(f"Evolution (Time: {t_arr[i]})", fontsize=24)

        p.set_offsets((q_history[i, 0], q_history[i, 1]))
        h[0].set_data(
            [q_history[i, 0], q_history[i, 0] + llen * np.cos(q_history[i, 2])],
            [q_history[i, 1], q_history[i, 1] + llen * np.sin(q_history[i, 2])],
        )
        pref.set_offsets((dr_history[i, 0], dr_history[i, 1]))
        href[0].set_data(
            [dr_history[i, 0], dr_history[i, 0] + rope[:, 0]],
            [dr_history[i, 1], dr_history[i, 1] + rope[:, 1]],
        )
        href[0].set_data(
            [dr_history[i, 0], dr_history[i, 0] - 0.8 * llen * u_world_history[i, 0]],
            [dr_history[i, 1], dr_history[i, 1] - 0.8 * llen * u_world_history[i, 1]],
        )
        pact.set_offsets((bt_history[i, 0], bt_history[i, 1]))

        ii[0] += int(1 / (dt * framerate))
        if ii[0] >= len(t_arr):
            print("Resetting animation!")
            ii[0] = 0

        else:
            return [p, h, pref, href, pact]

    # Run animation
    ani = animation.FuncAnimation(
        fig6, func=update_ani, interval=dt * 1000 / speedup, save_count=100
    )
    plt.legend()
    plt.show()

    #    # Store data
    if store_data:
        from scipy.io import savemat

        filename = f"data_{path_type}_{T}s"
        data = {
            "time": t_arr,
            "north": q_history[:, 0],
            "east": q_history[:, 1],
            "heading": q_history[:, 2],
            "surge": q_history[:, 3],
            "sway": q_history[:, 4],
            "yawrate": q_history[:, 5],
            "north_des": dr_history[:, 0],
            "east_des": dr_history[:, 1],
            "heading_des": dr_history[:, 2],
            "surge_des": dr_history[:, 3],
            "sway_des": dr_history[:, 4],
            "yawrate_des": dr_history[:, 5],
            "surge_force": u_history[:, 0],
            "sway_force": u_history[:, 1],
            "yaw_torque": u_history[:, 2],
            "rudder_surge_force": u_rudder_history[:, 0],
            "rudder_sway_force": u_rudder_history[:, 1],
            "rudder_yaw_torque": u_rudder_history[:, 2],
        }
        savemat(filename, data)
        print("Data saved!\n")

    plt.style.use("default")
    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "sans-serif",
        }
    )
    plt.rcParams.update({"font.size": 18})

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(1, 1, 1)
    ax2.plot(q_history[:, 0], q_history[:, 1], "--k", label="Boat Traj.")
    ax2.plot(q_history[0, 0], q_history[0, 1], "ok", label="Boat Start Point")
    # ax2.plot(q_history[-1,0],q_history[-1,1], 'sk', label='Boat End Point')
    ax2.plot(dr_history[:, 0], dr_history[:, 1], "--b", label="Drone Traj.")
    ax2.plot(dr_history[0, 0], dr_history[0, 1], "ob", label="Drone Start Point")
    # ax2.plot(dr_history[-1,0],dr_history[-1,1], 'sb', label='drone final position')
    ax2.set_xlabel("$x_I$ (m)")
    ax2.set_ylabel("$y_I$ (m)")
    ax2.grid(True)
    plt.legend()
    plt.show()
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(1, 1, 1)
    ax3.plot(q_history[:, 0], q_history[:, 1], "--k", label="Boat Traj. in Sim.")
    ax3.plot(bt_history[:, 0], bt_history[:, 1], "-r", label="Boat Traj. in Exp.")
    #if rudder_control != "none" and rudder_control != "step":
        #ax3.plot(ref[:, 0], ref[:, 1], "-b", label="Boat reference Traj")  #see which trajectory to follow
    ax3.plot(
        q_history[0, 0], q_history[0, 1], "ok", label="Start Point (Sim. and Exp.)"
    )
    ax3.set_xlabel("$x_I$ (m)")
    ax3.set_ylabel("$y_I$ (m)")
    ax3.grid(True)
    plt.legend()
    plt.show()

    # This part of the code get rid of the error that's caused by time delay and only measures how close the boat is to the trajectory
    ref = get_boat_reference()
    ref = ref[:int(len(ref)/2)]
    count = 0
    error = []
    error_x = []
    error_y = []
    for ii in ref:
        dx = q_history[:, 0] - ii[0]
        dy = q_history[:, 1] - ii[1]
        d = np.hypot(dx, dy)
        error.append(min(d))
        error_x.append(min(abs(dx)))
        error_y.append(min(abs(dy)))
        count += 1
    print("Mean Error: ", np.mean(error))
    print("Mean Error X: ", np.mean(error_x))
    print("Mean Error Y: ", np.mean(error_y))


    print(
            "Mean Square Error X: ", np.mean((q_history[:, 0] - bt_history[:, 0]) ** 2)
        )
    print(
            "Mean Square Error Y: ", np.mean((q_history[:, 1] - bt_history[:, 1]) ** 2)
        )
