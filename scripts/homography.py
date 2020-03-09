#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import cv2

#this is a ROS node that will locate a cone via homography transformation
class Homography():

    def __init__(self):
        #initialize variables
        print "hello"
        self.bridge = CvBridge()
        self.message_frame = "map"

        self.src_points = np.array([[0.775,0.055], [0.925, 0.055], [0.925, -0.25], [0.775, -0.25]])
        self.dest_points = np.array([[366.0, 223.0], [366.0, 213.0], [475.0, 211.0], [495.0, 219.0]])
        self.homography_mat = cv2.findHomography(self.src_points,self.dest_points)

        #subscribe to camera
        self.cam_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color",
            Image, self.cam_callback)
        #subscribe to clicking
        self.click_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color_mouse_left",
             Point, self.click_callback)
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
        marker.scale.x = 2000
        marker.scale.y = 2000
        marker.scale.z = 2000
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1
        self.marker_pub.publish(marker)

    def click_callback(self, point):
        x = point.x
        y = point.y
        coords = np.dot(self.homography_mat[0], np.array([[x], [y], [1]]))
        #normalizing over z
        coords = coords/coords[2]
	self.draw_marker(coords[0], coords[1], 1)
        print "Marker placed at: " +str(coords[0]) + str(coords[1])

if __name__ == '__main__':
    try:
        rospy.init_node('homography', anonymous=True)
        Homography()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
