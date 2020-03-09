#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import cv2

#this is a ROS node that will locate a cone via homography transformation
class Homography():

    def _init_(self):
        #initialize variables
        self.bridge = Cv_Bridge()
        self.message_frame = "map"

        self.src_points = np.array([[599.0, 228.0], [152.0, 231.0], [374.0, 231.0], [475.0, 195.0], [249.0, 195.0], [363.0, 195.0], [426.0, 179.0], [291.0, 181.0], [359.0, 181.0]])
        self.dest_points = np.array([[36.0, -24.0], [36.0, 24.0], [36.0, 0.0], [72.0, -24.0], [72.0, 24.0], [72.0, 0.0], [120.0, -24.0], [120.0, 24.0], [120.0, 0.0]])/39.37
        self.homography_mat = cv2.findHomography(self.src_points,self.dest_points)

        #subscribe to camera
        self.cam_sub = rospy.Subscriber("/zed/rgb/image_rect_color",
            Image, self.cam_callback)
        #subscribe to clicking
        self.click_sub = rospy.Subscriber("/zed/rgb/image_rect_color_mouse_left",
             PointStamped, self.click_callback)
        #publish marker for mouse clicks
        self.marker_pub = rospy.Publisher('/clicked_point_marker',
            Marker,queue_size=10)


    def cam_callback(self,data):
        try:
            cv = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            print(CvBridgeError)

    def draw_marker(self, x,y,z):
        marker = Marker()
        marker.header.frame_id = self.message_frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        self.marker_pub.Publish(marker)


    def click_callback(self,point):
        x = data.x
        y = data.y
        coords = np.dot(self.h[0], np.array([[x], [y], [1]]))
		coords = coords/coords[2]
		self.draw_marker((coords[0], coords[1]))

if __name__ == '__main__':
    try:
        rospy.init_node('homography', anonymous=True)
        Homography()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
