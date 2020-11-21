from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # Car parameters are defined in dbw_node.py
        
        ##** Steering angle Yaw controller **##
        
        #min_speed = 0.1 [m/s]
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        ##** Throttle PID controller **##
        
        # PID coefficients
        kp = 0.3
        ki = 0.1
        kd = 0.
        # Throttle limits
        mn = 0.
        mx = 0.2
        
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        # LPF (filtering out high frequency noise in the velocity)
        
        # f0 = 1/(2*pi*tau)
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()
        
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):

        
        if not dbw_enabled: 
            # Resetting controller when dbw is disabled, to avoid error
            # accumulation problem in PID controller
            # returns zeros to enable manual control
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        # Steering using get_steering method of YawController class  
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # Filtering current_vel
        current_vel = self.vel_lpf.filt(current_vel)
        
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        # Throttling using step method of PID class  
        throttle = self.throttle_controller.step(vel_error, sample_time)
        
        
        ##** Braking routine **##
        brake = 0
        
        # If target velocity (linear_vel = 0) and current_vel is low, then
        # vehicle is trying to stop by setting throttle to zero and forcing 
        # high brake for a complete stop
        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = 400
        
        # If vel_error is negative (exceeding target linear_vel) and throttle
        # PID control stopped decrement although vehicle needs to slow down
        elif throttle < 0.1 and vel_error < 0.0:
            throttle = 0
            
            # deceleration is dependent on vel_error
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
            
        
        return throttle, brake, steering
