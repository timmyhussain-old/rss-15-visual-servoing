#!/usr/bin/env python
import rospy
from lab4.msg import cone_location, parking_error
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
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

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        
        #################################
        # Play with this number too
        parking_distance = .75 #meters
        
        # Your Code Here.
        # Use relative position and your control law to populate
        # drive_cmd.
        
        #################################
        self.current_distance = np.sign(self.relative_x)*np.linalg.norm([self.relative_x, self.relative_y])
        current_angle = np.arctan2(self.relative_y, self.relative_x) 

        if (self.current_distance - parking_distance) < 0 or (abs(self.current_distance - parking_distance) < 0.5 and abs(current_angle) > 1e-1):
            self.back_up = True

        if(self.back_up):
            print "Backing up"
            drive_velocity = -1
            wheel_angle = np.sign(drive_velocity)*current_angle
            if (abs(current_angle) < 1e-1):
                self.back_up = False

        else:
            drive_velocity = (self.current_distance - parking_distance)
            wheel_angle = np.sign(drive_velocity)*current_angle

        rospy.loginfo(self.current_distance)
        rospy.loginfo(current_angle)


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

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
