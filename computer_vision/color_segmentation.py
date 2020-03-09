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
        
        # Convert BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # define range of orange color in HSV
        lower_orange = np.array([3,100,100])
        upper_orange = np.array([18,255,255])

        # Threshold the HSV image to get only orange colors
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

        cont_sorted = sorted(contours, key=cv2.contourArea, reverse=True)
    
        x,y,w,h = cv2.boundingRect(cont_sorted[0])
    
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),5)
        image_print(img)

	bounding_box = ((x,y),(x+w,y+h))

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box
