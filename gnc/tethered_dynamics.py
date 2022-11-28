#!/usr/bin/env python3
"""
Contains a model of a 3DOF marine ship. Supplies parameter values,
a function for the full nonlinear dynamic. Running this script as
__main__ will present a quick open-loop simulation.

Boat State is: [x_world_position (m),
                y_world_position (m),
                heading_angle_from_x (rad),
                x_body_velocity (m/s),
                y_body_velocity (m/s),
                yaw_rate (rad/s)]

Force u is: [x_body_force (N),
             y_body_force (N),
             z_torque (N*m)]

Input dr is: [drone_x_position (m),
              drone_y_position (m)]

See:
T. I. Fossen, Handbook of Marine Craft Hydrodynamics
and Motion Control. Wiley, 2011. Chapter 13.

Partial code taken from https://github.com/jnez71/misc
"""
import numpy as np

npl = np.linalg

# Bathydrone specific mass, inertia and dimensions
m = 6.342  # kg
xg = 0  # COM x coord in bocy fixed frame [m]
Iz =  4  # mass moment of intertia from SolidWorks 
# Offset on the tension application point [m]
off_x = 0.1905  # measured from the bathydrone vehicle
off_z = 0.1016
r = np.array([off_x, 0, off_z])
# height of drone above water
hd = 20*0.681818  # feet to meters
le = 31*0.681818  # Length of rope
proj_le = np.sqrt(le**2-hd**2)
dL = 1  # tolrance lentgh for rope constraint

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

# Inertial matrix, independent of state
M = np.array([
              [m - wm_xu,            0,            0],
              [        0,    m - wm_yv, m*xg - wm_yr],
              [        0, m*xg - wm_yr,   Iz - wm_nr]
            ])
Minv = npl.inv(M)


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

class TetheredDynamics():
    def __init__(self, ten_mag) -> None:
        self.dr = np.array([0, 0, 0], dtype=np.float64)  
        self.ten = np.array([0, 0, 0], dtype=np.float64)  
        self.u = np.array([0, 0, 0], dtype=np.float64)  
        self.diff_pos = np.array([0, 0], dtype=np.float64)  
        self.ten_mag = ten_mag
        self.hd = hd
        self.proj_le = proj_le
        self.mult = 1
        self.dL = dL
        
    def step(self, q, v_dr, dt):
        """
        Dynamic model of the boat. Returns the derivatives of the state q based on the control input u
        Input: 
            q: State of the boat. Vector of position, orientation and velocites
            v_dr: drone velocity
        Output:
            q (t+1): State of the boat at the next time step 
        """
        if npl.norm(q[:2] - self.dr[:2]) >= self.proj_le+dL:
            v_dr = np.array([0, 0, 0])

        R = get_R(q)
        self.dr[:2] = self.dr[:2] + v_dr[:2] * dt
        app_point = np.array([q[0], q[1], 0]) + R.dot(r) # world coord
        self.diff_pos = self.dr - app_point
        self.diff_pos = self.diff_pos/npl.norm(self.diff_pos)

        self.ten = self.ten_mag*self.diff_pos
        ten_body = R.T.dot(self.ten)
        moments = np.cross(r, ten_body)
        self.u[:2] = ten_body[:2]
        self.u[2] = moments[2]  # apply moment about z
        self.ten[2] = moments[2]

        if npl.norm(q[:2] - self.dr[:2]) < proj_le-dL:
            self.u = np.array([0, 0, 0])
        # Centripetal-coriolis matrix
        C = np.array([
                      [                                     0,                0, (wm_yr - m*xg)*q[5] + (wm_yv - m)*q[4]],
                      [                                     0,                0,                       (m - wm_xu)*q[3]],
                      [(m*xg - wm_yr)*q[5] + (m - wm_yv)*q[4], (wm_xu - m)*q[3],                                      0]
                    ])

        # Drag matrix
        D = np.array([
                      [-d_xuu*abs(q[3])*self.mult,                                    0,                                    0],
                      [               0, -(d_yvv*abs(q[4]) + d_yrv*abs(q[5])), -(d_yvr*abs(q[4]) + d_yrr*abs(q[5]))],
                      [               0, -(d_nvv*abs(q[4]) + d_nrv*abs(q[5])), -(d_nvr*abs(q[4]) + d_nrr*abs(q[5]))]
                    ])

        R = get_R(q)

        # EQUATIONS OF MOTION
        # M*vdot + C*v + D*v = u  and  pdot = R*v

        # time rate of change of the state. Velocities and acceleration 
        dyn = np.concatenate((R.dot(q[3:]), Minv.dot(self.u - (C + D).dot(q[3:]))))
        return qplus(q, dyn*dt)