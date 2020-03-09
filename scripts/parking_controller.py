#!/usr/bin/env python
import rospy
from lab4.msg import cone_location, parking_error
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from tf import TransformListener
from geometry_msgs.msg import Point, PointStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    #Vehicle Wheel Base
    L = 0.5
    l_fw = L/2
    PARKING_DISTANCE = 0.75
    SPEED_LIMIT = 5
    def __init__(self):
        rospy.Subscriber("/relative_cone", cone_location, 
            self.relative_cone_callback)    
        self.drive_pub = rospy.Publisher("/drive", 
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            parking_error, queue_size=10)

        self.relative_x = 0
        self.relative_y = 0
        self.current_distance = 0
        self.back_up = False

        self.pid_distance = PID(5, 0, 0, setpoint= self.PARKING_DISTANCE, output_limits = (-self.SPEED_LIMIT, self.SPEED_LIMIT))

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        
        #################################
        # Play with this number too
        
        # Your Code Here.
        # Use relative position and your control law to populate
        # drive_cmd.
        
        #################################
        
        self.current_distance = np.linalg.norm([self.relative_x, self.relative_y])

        x = self.relative_x - self.l_fw #quick transformation to rear wheel point
        y = self.relative_y

        L_fw = np.linalg.norm([x, y])

        eta = np.arctan2(y, x) 
        phi = np.arctan2(self.L*np.sin(eta), L_fw/2 + self.l_fw*np.cos(eta))

        drive_velocity = -self.pid_distance(self.current_distance) 
        wheel_angle = np.sign(drive_velocity)*phi

        if drive_velocity < 1e-1 and abs(phi) > 1e-1: 
            self.back_up = True

        if(self.back_up):
            drive_velocity = -1.5*np.abs(wheel_angle)
            wheel_angle = np.sign(drive_velocity)*phi
            if (np.abs(phi) < 1e-1):
                self.back_up = False

        drive_cmd.header.frame_id = "base_link"
        drive_cmd.header.stamp = rospy.get_rostime()
        drive_cmd.drive.speed = drive_velocity 
        drive_cmd.drive.steering_angle = wheel_angle
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()
        
    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = parking_error()
        
        #################################
        
        # Your Code Here
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = self.current_distance 

        #################################
        self.error_pub.publish(error_msg)

"""
    Limits a value to be within the limits
"""
def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif upper is not None and value > upper:
        return upper
    elif lower is not None and value < lower:
        return lower
    return value

"""
    Generic PID controller class
"""
class PID:
    def __init__(self,
                 Kp=1.0, Ki=0.0, Kd=0.0,
                 setpoint=0,
                 output_limits=(None, None)):
        self._Kp, self._Ki, self._Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self._output_limits = output_limits
        self.reset()

    def __call__(self, input_):

        current_time = rospy.get_time()
        dt = current_time - self._last_time if current_time - self._last_time else 1e-16

        # compute error terms
        error = self.setpoint - input_
        d_error = error - (self._last_error if self._last_error is not None else error)

        # regular proportional-on-error, simply set the proportional term
        self._proportional = self._Kp * error

        # compute integral and derivative terms
        self._integral += self._Ki * error * dt
        self._integral = _clamp(self._integral, self._output_limits)  # avoid integral windup

        self._derivative = self._Kd * d_error / dt

        # compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self._output_limits)

        # keep track of state
        self._last_output = output
        self._last_error = error
        self._last_time = current_time

        return output
    
    def reset(self):
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = rospy.get_time()
        self._last_output = None
        self._last_error = None






if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
