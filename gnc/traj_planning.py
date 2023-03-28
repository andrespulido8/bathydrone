#!/usr/bin/env python3
""" Made to chain the rrt planning for many goals but it is not
    working yet. I might try to use this file later again
    when I try to do online planning

    In boat.py I tried:

    from traj_planning import pdRRT_Node

    # rrt = pdRRT_Node()
    # TRAJ PLANNER IN THE SIM LOOP
    # rrt_result = rrt.action(traj[goal_counter, :], q, is_same_goal, t)
    # if not rrt_result:
    #    is_same_goal = True
    #    q_ref = rrt.get_ref(t)
    #    q_ref_dot = q_ref[3:]

    # else:
    #    is_same_goal = False
    #    goal_counter += 1
    #    print("=" * 50)
    #    #for tt in range(traj.shape[0]):  TODO: uncomment
    #    if goal_counter == 1:
    #        break
"""
import time
from typing import List, Optional

import lqrrt
import numpy as np
import numpy.linalg as npl

# Check scipy version for assume_sorted argument in interp1d
import scipy.interpolate
from path_planning import pp
from tethered_dynamics import TetheredDynamics

if int(scipy.__version__.split(".")[1]) < 16:

    def interp1d(*args, **kwargs):
        kwargs.pop("assume_sorted", None)
        return scipy.interpolate.interp1d(*args, **kwargs)

else:
    interp1d = scipy.interpolate.interp1d

# INITIALIZATIONS
nstates = 6
ncontrols = 3

real_tol = [2, 2, np.deg2rad(20), np.inf, np.inf, np.inf]
pointshoot_tol = np.deg2rad(20)  # rad
free_radius = 6  # m
basic_duration = 1  # s
fudge_factor = 0.85

################################################# ISSUE CONTROL

stuck_threshold = 2  # moves
fail_threshold = 5  # stucks
collision_threshold = 0.5  # s
reeval_time = 60  # s
reeval_limit = 1000  # iterations
focus = None

################################################# TREE GROWTH

horizon = (0.1, 3)  # s
dt = 0.1  # s
FPR = 0
ss_start = 10  # m
ss_step = 5  # m
max_nodes = 1e5

kp = np.array([0.2, 0.2, 0])
ki = np.array([0.01, 0.01, 0.0])
kd = np.array([0.0, 0.0, 0.0])
# Vehicle dimensions
boat_length = 1  # m
boat_width = 0.5  # m
goal_buffer = [2, 2, np.inf, np.inf, np.inf, np.inf]
error_tol = np.copy(goal_buffer) / 2
obs = []
obs = np.array(
    [
        [3, 2, 2],
    ]
)
goal_bias = [0.3, 0.3, 0, 0, 0, 0]
goal = [10, 10, np.deg2rad(90), 0, 0, 0]


def is_feasible(x, u):
    for ob in obs:
        if npl.norm(x[:2] - ob[:2]) <= boat_length / 2 + ob[2]:
            return False
    return True


def lqr(x, u):
    """Returns cost-to-go matrix S and policy matrix K given local state x and effort u."""
    global kp, kd
    kp_mat = np.diag(kp)
    kd_mat = np.diag(kd)
    S = np.diag([1, 1, 0.1, 0.1, 0.1, 0.1])
    K = np.hstack((kp_mat, kd_mat))
    return (S, K)


def gen_ss(seed, goal, buff=[ss_start] * 4):
    """
    Returns a sample space given a seed state, goal state, and buffer.

    """
    return [
        (min([seed[0], goal[0]]) - buff[0], max([seed[0], goal[0]]) + buff[1]),
        (min([seed[1], goal[1]]) - buff[2], max([seed[1], goal[1]]) + buff[3]),
        (-np.pi, np.pi),
        (-abs(0), 2),
        (-abs(0.2), 0.2),
        (-abs(0.8), 0.8),
    ]


unset = None
constraints = lqrrt.Constraints(
    nstates=nstates,
    ncontrols=ncontrols,
    goal_buffer=goal_buffer,
    is_feasible=is_feasible,
)
dynamics = TetheredDynamics(ten_mag=9)
planner = lqrrt.Planner(
    dynamics.step,
    lqr,
    constraints,
    horizon=horizon,
    dt=dt,
    FPR=FPR,
    error_tol=error_tol,
    erf=unset,
    min_time=basic_duration,
    max_time=3,
    max_nodes=max_nodes,
    sys_time=unset,
    printing=False,
)


class pdRRT_Node:
    """
    Example of a ROS node that uses lqRRT for a big boat.

    This node subscribes to the current boat state (Odometry message),
    and the world-frame ogrid (OccupancyGrid message). It publishes the
    REFerence trajectory that moves to the goal, as an Odometry message.
    It provides an action for moving said reference to that goal (Move.action).

    Attributes:
        revisit_period (float): When to refresh the core of the planner.
        ref_pub (rospy.Publisher): The publisher serving the lqRRT reference
            topic name.
        path_pub (rospy.Publisher): Publisher responsible for publishing a PoseArray
            to the provided path topic.
        tree_pub (rospy.Publisher): Publisher responsible for publishing a PoseArray
            to the provided tree topic.
        unreachable (bool): Whether the planner sees a goal as being unreachable.
            Defaults to ``false``.
        done (bool): Whether the planner has finished its movement. Defaults to ``false``.
        move_type (MoveAction): How the boat is planning to move towards its goal.
    """

    def __init__(
        self,
    ):
        """
        Initialize with topic names
        """
        # One-time initializations
        self.state = None
        self.tracking = None
        self.done = True

        # Lil helpers
        self.intup = lambda arr: tuple(np.array(arr, dtype=np.int64))

        # Set-up planners
        planner.set_system(erf=self.erf)
        planner.set_runtime()
        planner.constraints.set_feasibility_function(is_feasible)

        # Initialize resettable stuff
        self.reset()

    def reset(self) -> None:
        """
        Resets variables that should definitely be cleared before a new action begins.
        """
        # Internal plan
        self.goal = None
        self.get_ref = None
        self.get_eff = None
        self.x_seq = None
        self.u_seq = None
        self.tree = None
        self.goal_bias = None
        self.sample_space = None
        self.guide = None
        self.speed_factor = np.array(1)

        # Planning control
        self.last_update_time = None
        self.next_runtime = None
        self.next_seed = None
        self.move_count = 0
        self.initial_plan_time = basic_duration

        # Issue control
        self.stuck = False
        self.stuck_count = 0
        self.fail_count = 0
        self.failure_reason = ""
        self.preempted = False
        self.unreachable = False
        self.collided = False

        self.lock_tree = False
        planner.unkill()

    # ACTION
    def action(self, goal, state, is_same_goal, t) -> Optional[bool]:
        """
        Returns:
            Optional[bool]: success
        """
        # TODO: reference trajectory is going crazy

        # Start clean
        self.done = False
        self.reset() if not is_same_goal else None

        self.set_goal(goal)
        self.state = state

        planner.dt = dt

        self.next_runtime = basic_duration
        self.next_seed = np.copy(self.state)

        # (debug)
        if not self.lock_tree:
            assert self.next_seed is not None
            assert self.next_runtime is not None

        # Begin tree-chaining loop
        clean_update = self.tree_chain(t)
        self.move_count += 1

        err_now = np.abs(self.erf(self.goal, self.state))
        # Print feedback
        if clean_update and not (
            self.preempted or self.unreachable or self.stuck or self.lock_tree
        ):
            print(f"\nMove {self.move_count}\n----")
            print(f"Goal bias: {np.round(self.goal_bias, 2)}")
            print(f"Tree size: {self.tree.size}")
            print(f"Move duration: {np.round(self.next_runtime, 1)}")
            print(f"Error: {err_now}")
        # elif not self.lock_tree:
        #     print("\nIssue Status\n----")
        #     print("Stuck: {}".format(self.stuck))
        #     print("Collided: {}".format(self.collided))
        #     print("Unreachable: {}".format(self.unreachable))
        #     print("Preempted: {}".format(self.preempted))
        #     print("Tree size: {}".format(self.tree.size))
        #     print("Move duration: {}".format(np.round(self.next_runtime, 1)))

        if np.all(err_now <= real_tol):
            remain = np.copy(self.goal)
            self.get_ref = lambda t: remain
            self.get_eff = lambda t: np.zeros(3)
            self.done = True
            print("\nDone!\n")
            return True

        if self.move_count > 400:
            self.preempted = True
            print("\nMove counter threshold reached\n")
            return True

        # Check for abrupt termination or unreachability
        if self.preempted or self.unreachable:
            remain = np.copy(np.concatenate((self.state[:3], np.zeros(3))))
            self.get_ref = lambda t: remain
            self.get_eff = lambda t: np.zeros(3)
            print("\nTerminated.\n")
            self.done = True
            return True

        return False

    # WHERE IT HAPPENS
    def tree_chain(self, t):
        """
        Plans an lqRRT and sets things up to chain along another lqRRT when
        called again.
        """
        # Make sure we are not currently in an update or a collided state
        if self.lock_tree:
            time.sleep(0.01)
            return False

        # Thread locking, lol
        self.lock_tree = True

        # No issue
        if (
            self.next_runtime < basic_duration
            and self.last_update_time is not None
            and not self.stuck
        ):
            self.next_runtime = basic_duration
            self.next_seed = self.get_ref(self.next_runtime + t - self.last_update_time)
        (
            self.goal_bias,
            self.sample_space,
            self.guide,
            self.pruning,
        ) = self.select_exploration()

        # Special first-move case
        if self.move_count == 0:
            if self.get_ref is not None:
                if self.initial_plan_time > self.next_runtime:
                    self.next_runtime = self.initial_plan_time
            else:
                self.next_runtime = self.initial_plan_time

        # Update plan
        clean_update = planner.update_plan(
            x0=self.next_seed,
            sample_space=self.sample_space,
            goal_bias=self.goal_bias,
            guide=self.guide,
            pruning=self.pruning,
            specific_time=self.next_runtime,
        )

        # Update finished properly
        if clean_update:
            # We might be stuck if tree is oddly small
            if planner.tree.size == 1:
                # Increase stuck count towards threshold
                self.stuck_count += 1
                if self.stuck_count > stuck_threshold:
                    self.fail_count += 1
                    if self.fail_count > fail_threshold:
                        print("\nNot making any progress.\nGoal may be unreachable.")
                        self.unreachable = True
                        self.failure_reason = "unreachable"
                    else:
                        print("\nI think we're stuck...")
                        self.stuck = True
                        self.stuck_count = 0
                else:
                    self.stuck = False
            else:
                self.stuck = False
                self.stuck_count = 0
                self.fail_count = 0

            # Cash-in new goods
            self.x_seq = np.copy(planner.x_seq)
            self.u_seq = np.copy(planner.u_seq)
            self.tree = planner.tree
            self.last_update_time = t
            self.get_ref = planner.get_state
            self.get_eff = planner.get_effort
            self.next_runtime = planner.T
            if self.next_runtime > basic_duration:
                self.next_runtime *= fudge_factor
            self.next_seed = self.get_ref(self.next_runtime)
            self.time_till_issue = None

        # Make sure all planners are actually unkilled
        planner.unkill()

        # Unlocking, lol
        self.lock_tree = False

        return clean_update

    def select_exploration(
        self,
    ) -> List:
        """
        Chooses the goal bias, sample space, guide point, and pruning
        choice for the current move and behavior.
        """
        b = 0.5
        gs = np.copy(self.goal)
        ss = gen_ss(self.next_seed, self.goal)

        return ([b, b, 0.3, 0, 0, 0.1], ss, gs, True)

    def reevaluate_plan(self) -> None:
        """
        Iterates through the current plan re-checking for feasibility
          and checking that the goal is still feasible.
        """
        # Make sure a plan exists and that we aren't currently fixing it
        last_update_time = self.last_update_time
        x_seq = np.copy(self.x_seq)
        if (
            last_update_time is None
            or not np.any(x_seq)
            or self.done
            or self.time_till_issue is not None
        ):
            return

        # Timesteps since last update
        iters_passed = int((self.rostime() - last_update_time) / dt)

        # Make sure that the goal pose is still feasible
        if not is_feasible(self.goal, np.zeros(3)):
            print("\nThe given goal is occupied!\nGoing nearby instead.")
            self.time_till_issue = np.inf
            self.failure_reason = "occupied"
            p_err = self.goal[:2] - self.state[:2]
            p_err_mag = npl.norm(p_err)
            if p_err_mag <= npl.norm(real_tol[:2]):
                print("...looks like I am already nearby.")
                self.set_goal(self.state)
                return
            npoints = int(p_err_mag / (boat_length / 2))
            xline = np.linspace(self.goal[0], self.state[0], npoints)
            yline = np.linspace(self.goal[1], self.state[1], npoints)
            hline = [np.arctan2(p_err[1], p_err[0])] * npoints
            sline = np.vstack((xline, yline, hline, np.zeros((3, npoints)))).T
            found_free_goal = False
            for i, x in enumerate(sline[:-1]):
                if is_feasible(x, np.zeros(3)):
                    self.set_goal(sline[i + 1])
                    found_free_goal = True
                    break
            if not found_free_goal:
                print("\nCould not find a reasonable free goal!")
                self.unreachable = True
                self.failure_reason = "unreachable"
                return
            for behavior in self.behaviors_list:
                behavior.planner.kill_update()
            return

        # Check that the next reeval_time seconds in the current plan are still feasible
        p_seq = np.copy(
            x_seq[
                iters_passed : min(
                    [
                        len(x_seq),
                        int(iters_passed + (reeval_time / dt)),
                        reeval_limit,
                    ]
                )
            ]
        )
        if len(p_seq):
            p_seq[:, 3:] = 0
            for i, (x, u) in enumerate(zip(p_seq, [np.zeros(3)] * len(p_seq))):
                if not is_feasible(x, u):
                    time_till_collision = i * dt
                    if time_till_collision <= collision_threshold:
                        if not self.collided:
                            print("\nCollided! (*seppuku*)")
                            print("But we cannot throw away our shot!")
                            self.collided = True
                            self.failure_reason = "collided"
                    else:
                        print(
                            "\nFound collision on current path!\nTime till collision: {}".format(
                                time_till_collision
                            )
                        )
                        self.time_till_issue = time_till_collision
                        for behavior in self.behaviors_list:
                            behavior.planner.kill_update()
                    return
        if self.collided:
            print("\nNo longer collided! (*unseppuku*)")
            self.collided = False
            if self.failure_reason == "collided":
                self.failure_reason = ""

        # No concerns
        self.time_till_issue = None

    # LIL MATH DOERS
    def erf(self, goal_state: List[float], state: List[float]) -> np.ndarray:
        """
        Returns error given two states goal_state and state.
        Angle differences are taken properly on SO2.

        Args:
            goal_state (List[float]): The goal state
            state (List[float]): The state to derive the error from

        Returns:
            np.ndarray: The error between the goal and passed state.
        """
        e = np.subtract(goal_state, state)
        e[2] = self.angle_diff(goal_state[2], state[2])
        return e

    def angle_diff(self, angle_goal: float, angle: float) -> float:
        """
        Takes an angle difference properly on SO2.

        Args:
            angle_goal (float): The goal angle.
            angle (float): The angle to grab the difference from.

        Returns:
            float: The difference between the desired and goal angle.
        """
        c = np.cos(angle)
        s = np.sin(angle)
        cg = np.cos(angle_goal)
        sg = np.sin(angle_goal)
        return np.arctan2(sg * c - cg * s, cg * c + sg * s)

    # PUBDUBS
    def set_goal(self, x: np.ndarray) -> None:
        """
        Publishes the goal state to the goal publisher.
        """
        # Create and establish goal
        self.goal = np.copy(x)
        planner.set_goal(self.goal)


def main():
    sp1 = [0, 0]
    sp2 = [120, 80]
    dy = 60
    is_plot = False
    planning = pp(sp1, sp2, dy, is_plot)
    traj = planning.trajectory(max_vel=2)

    st = np.array([5, 5, np.deg2rad(45), 0, 0, 0])

    rrt = pdRRT_Node()
    is_same_goal = False

    # for tt in range(traj.shape[0]):
    for tt in range(1):
        is_same_goal = False
        print("=" * 50)
        while not rrt.action(
            goal=traj[tt, :], state=st, is_same_goal=is_same_goal, t=tt
        ):
            is_same_goal = True
            # Best controller you will ever see
            st = st + (traj[tt, :] - st) * 0.2
            print("state: ", st)
            print("goal: ", traj[tt, :])
            print("goal number: ", tt)

    print("FINISHED WITH ALL THE TREE!")

    return 0


if __name__ == "__main__":
    main()
