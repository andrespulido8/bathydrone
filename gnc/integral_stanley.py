import numpy as np
import matplotlib.pyplot as plt 
from math import atan, atan2, sqrt, pi


class IntegralStanley:

    def __init__(self, angular_gain=2.5, control_gain = 2.5, Integral_gain = 2, max_steer=np.deg2rad(24), reference=None, x_previous=0, y_previous=0, debug = False):
        
        """
        Integral Stanley

        At initialisation
        :param angular_gain:                (float) gain between angular error and control output 
        :param control_gain:                (float) time constant [1/s]
        :param max_steer:                   (float) vehicle's steering limits [rad]
        :param reference:                   (numpy.array) an 2 by n array that have list of x- and y-coordinates along the reference path
        :param x_previous:                  (float) previous x position for vehicle
        :param y_previous:                  (float) previous y position for vehicle
        :param debug:                       (boolean) whether to print all variables to the console

        At every time step
        :param x:                           (float) vehicle's x-coordinate [m]
        :param y:                           (float) vehicle's y-coordinate [m]
        :param dt                           (float) change of time between two times the stanley controller is called

        :return limited_steering_angle:     (float) steering angle after imposing steering limits [rad]
        :return crosstrack_error:           (float) distance from closest path index [m]
        :return steering_error:             (float) the heading error after multiply with gain 
        :return heading_error:              (float) the heading error from the boat
        """

        self.alpha = angular_gain 
        self.k = control_gain
        self.ki = Integral_gain
        self.max_steer = max_steer

        self.rx = reference[:,0]
        self.ry = reference[:,1]
        self.x_p = x_previous
        self.y_p = y_previous

        self.debug = debug

        self.current_time = 0
        self.previous_time = 0
        self.phi_integral = 0  #used for integration of heading error
        self.phi_p = 0  #previous heading error
        self.e_integral = 0  #used for integration of crosstrack error
        self.e_p = 0  #previous crosstrack error

    
    def find_target_path_id(self, x, y):  
        """Finds if of the closest waypoint in the reference path from the current pose"""
        dx = self.rx - x   # Find the x-axis of the position relative to the path
        dy = self.ry - y   # Find the y-axis of the position relative to the path

        d = np.hypot(dx, dy) # Find the distance from the current position to the all points in the path
        target_index = np.argmin(d) # Find the shortest distance in the array

        return target_index, dx[target_index], dy[target_index], d[target_index]
    
    def find_current_headingVelocity(self, x, y, dt): 
        """Estimates current heading and velocity given the current and previous position """
        dx = x - self.x_p
        dy = y - self.y_p  #find the distance travelled between the two time steps
        current_heading = atan2(dy,dx)
        current_velocity = sqrt((dx**2) + (dy**2))/dt

        return current_heading, current_velocity
    
    def update_position(self,x,y):
        """update previous coordinate with current coordinate to allow the next calculation""" 
        self.x_p = x
        self.y_p = y  
    
    def get_distance(self, lat1, lon1, lat2, lon2): 
        """Returns the distance between two points, convert from latitude/logitude to meters"""
        lat1 = np.deg2rad(lat1)
        lon1 = np.deg2rad(lon1)
        lat2 = np.deg2rad(lat2)
        lon2 = np.deg2rad(lon2)
        R = 6_378_100
        dx = (lon2 - lon1) * np.cos(0.5*(lat2+lat1))
        dy = lat2 - lat1
        return R * np.sqrt(dx**2 + dy**2), dx, dy

    def find_reference_heading(self, target_index): 
        """Finds the heading of the reference path at the closest waypoint"""
        dx, dy = 0, 0
        jj = 0
        while dx == 0 or dy == 0 and target_index+jj < self.rx.size-1:  # this while loop make sure there is a change in reference heading
            jj += 1
            if target_index == self.rx.size-1:  #account for edge case when the target_index is at very end, it will be out of bound 
                dx = self.rx[target_index] - self.rx[target_index-jj]
                dy = self.ry[target_index] - self.ry[target_index-jj]
            else: 
                dx = self.rx[target_index+jj] - self.rx[target_index]
                dy = self.ry[target_index+jj] - self.ry[target_index]  #find the vector pointing from closest waypoint to the next waypoint
        reference_heading = atan2(dy,dx)   #find the angle of that vector 

        return reference_heading
    
    def find_steering_error(self, current_heading, reference_heading): 
        phi = reference_heading - current_heading
        
        #account for angle wrapping 
        if phi < -pi: 
            phi = phi + 2*pi
        elif phi > pi:
            phi = phi - 2*pi

        return - self.alpha*phi, phi
    
    def crosstrack_steering_angle(self, dx, dy, d, current_velocity, reference_heading): 
        """
        Calculates the cross-track error output
        :param dx:                  (float) difference on x-axis between closest reference point and current position, given in find_target_path_id() function
        :param dx:                  (float) difference on y-axis between closest reference point and current position, given in find_target_path_id() function
        :param d:                   (float) distance between current position and closest waypoint, used as cross track error
        :param current_velocity:    (float) current velocity estimated from GPS coordinates 
        :param reference_heading:   (float) the heading of the reference position, used to determine sign of cross track error term, see documentation
        """

        theta = atan2(dy,dx) - reference_heading

        #account for angle wrapping 
        if theta < -pi: 
            theta = theta + 2*pi
        elif theta > pi:
            theta = theta - 2*pi

        #so there is no division by 0 in the controller 
        if current_velocity == 0:
            current_velocity = 1
        
        return - np.sign(theta)*atan(self.k*d/current_velocity)   #return the cross track error term with correct sign, indicated by derivation in documentation
    
    def stanley_control(self, x, y, dt): 
        """Returns the desired rudder angle to be commanded"""
        x = x
        y = y
        dt = dt  #update time and dt value 
        #dt should never be 0 since the time is almost never 0

        target_index, dx, dy, d = self.find_target_path_id(x, y)
        current_heading, current_velocity = self.find_current_headingVelocity(x,y,dt)
        reference_heading = self.find_reference_heading(target_index)

        steering_error, heading_error = self.find_steering_error(current_heading,reference_heading)
        crosstrack_error = self.crosstrack_steering_angle(dx,dy,d,current_velocity,reference_heading)
        #self.update_position(x,y)    '''temporarily removed for plotting purposes'''

        self.phi_integral += self.phi_p*dt
        self.e_integral += self.e_p*dt  #update the integral term for both heading and crosstrack error
        self.phi_p = heading_error
        self.e_p = crosstrack_error  #update the previous error term for both heading and crosstrack error

        control_angle = steering_error+ (-crosstrack_error) + self.ki*self.phi_integral + (-self.e_integral)  #seem to make the boat follow reference better
        # Make sure the equation is the same as the equation from the report 

        #can comment this part of the code to debug, give insight to where the angle will be at
        if control_angle > self.max_steer:
            control_angle = self.max_steer
        elif control_angle < -self.max_steer:
            control_angle = -self.max_steer
        #if the control output is greater than the max output of the rudder, then set the rudder to the max
        return np.rad2deg(control_angle), crosstrack_error, steering_error, heading_error 

        #this code will print various error to the ros topic 
        
        if self.debug == True: 
            print(f'The current position is x = {x}, y = {y}')
            print(f'Target index is {target_index}, dx = {dx}, dy = {dy}, d = {d}')
            print(f'The control angle is {round(np.rad2deg(control_angle),2)}, current heading = {round(np.rad2deg(current_heading),2)}, reference heading = {round(np.rad2deg(reference_heading),2)}, current velocity = {round(current_velocity,2)}m/s')
            print(f'The time change is {round(dt,2)}')

            print()

        #return control_angle
            
    def print_debug(self, x, y, dt):
        target_index, dx, dy, d = self.find_target_path_id(x, y)
        current_heading, current_velocity = self.find_current_headingVelocity(x,y,dt)
        reference_heading = self.find_reference_heading(target_index)

        return d, dx, dy, current_heading, reference_heading, current_velocity

    
    
def main():
    r = np.array([[29.638899,-82.358131], [29.638899,-82.358197], [29.638899,-82.358275], [29.638899,-82.358326], 
                  [29.638899,-82.358379], [29.638899,-82.358434], [29.638899,-82.358491], [29.638899,-82.358548], 
                  [29.638899,-82.358612], [29.638899, -82.358687]]) #define the reference points of the controller
    
    debug = True 
    
    Controller = IntegralStanley(angular_gain=1, control_gain = 2.5, reference=r ,x_previous=0.1-0.1*sqrt(2),y_previous=0.1,debug = debug)

if __name__ == '__main__':
    main()