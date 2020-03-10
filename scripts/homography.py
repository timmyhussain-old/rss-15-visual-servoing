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
        self.coords = []

        #points from other people
        #self.src_points =  np.array([[[387.0,375.0]],[[348.0,272.0]],[[336.0,240.0]],[[331.0,222.0]],[[555.0,273]],[[406.0,225.0]],[[223.0,253.0]],[[249.0,244.0]]])
        #self.dest_points  = np.array([[[0.3048,0.0]],[[0.6096,0.0]],[[0.9144,0.0]],[[1.1938,0.0]],[[0.6207,-0.37465]],[[1.108,-0.2381]],[[0.1859,0.2635]],[[0.8937,0.2190]]])

        #room calibration src_points
        self.src_points =  np.array([[[412.0,289.0]],[[314.0,216.0]],[[486.0,254.0]],[[393.0,235.0]],[[347.0,227.0]],[[479.0,223.0]],[[424.0,219.0]],[[388.0,215.0]]])
        self.dest_points  = np.array([[[0.474,0.0]],[[0.53,0.06]],[[0.62,0.17]],[[0.83,0.0]],[[0.94,0.12]],[[0.92,0.25]],[[1.02,0.11]],[[1.12,0]]])

        #self.src_points = np.array([[0.775,0.055], [0.925, 0.055], [0.925, -0.25], [0.775, -0.25]])
        #self.dest_points = np.array([[366.0, 223.0], [366.0, 213.0], [475.0, 211.0], [495.0, 219.0]])
        self.homography_mat = cv2.findHomography(self.src_points,self.dest_points)
        #the first value of the findHomography function is actually what we need
        self.homography_mat = self.homography_mat[0]
        print "homography_mat is: " + str(self.homography_mat)

        x_coord, y_coord, z_coord = np.matmul(self.homography_mat, np.vstack([458.0,279.0,0.0]))

        print "Marker Placed at: (" +str(x_coord[0]) + "," + str(y_coord[0]) + ")"

        #subscribe to camera
        self.cam_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color",
            Image, self.cam_callback)
        #subscribe to clicking
        self.click_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color_mouse_left",
             Point, self.click_callback)
        #publish marker for mouse clicks
        self.marker_pub = rospy.Publisher('/clicked_point_marker',
            Marker,queue_size=1)
        self.draw_marker(x_coord[0],y_coord[0],z_coord[0])
        print "what"

        #we woukld need extra publisher and subscriber for when we implement to
        #entire system
        #self.point_pub = rospy.Publisher("/homography/point", Point, queue_size=10)
        #subscribe to points from cone_locator.py
        #rospy.Subscriber("yadayada", Point, self.point_callback)

    def cam_callback(self,data):
        print "in cam_callback"
        try:
            cv = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            print(CvBridgeError)

    def draw_marker(self, x,y,z):
        print "drawing"
        marker = Marker()
        marker.header.frame_id = self.message_frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.scale.x = 100
        marker.scale.y = 100
        marker.scale.z = 100

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        self.marker_pub.publish(marker)



    def click_callback(self,point):
        print "in click_callback"
        x = point.x
        y = point.y
        z = point.z
        x_coord, y_coord, z_coord = np.matmul(self.homography_mat, np.vstack([x,y,z]))
        self.draw_marker(x_coord[0],y_coord[0],z_coord[0])
        print "Marker Placed at: (" +str(x_coord[0]) + "," + str(y_coord[0]) + ")"


if __name__ == '__main__':
    try:
        rospy.init_node('homography', anonymous=True)
        Homography()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
