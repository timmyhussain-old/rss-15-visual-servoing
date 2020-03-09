import cv2
import imutils
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
# 
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########

	bounding_box = ((0,0),(0,0))
	
	# Convert BGR to HSV
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of orange color in HSV
	lower_orange = np.array([20,50,50])
	upper_orange = np.array([40,255,255])

	# Threshold the HSV image to get only orange colors
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

	#erosion
	kernel = np.ones((5,5),np.uint8)
	erosion = cv2.erode(mask,kernel,iterations = 1)

	#dilation
	dilation = cv2.dilate(mask,kernel,iterations = 1)

	#find contours 
	contours,hierarchy = cv2.findContours(dilation, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

	cnt = contours[0]

	#bounding box
	x,y,w,h = cv2.boundingRect(cnt)
    bounding_box=((x,y),(x+w,y+h))




	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box
