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
	Helper function to print out images, for debugging.
	Press any key to continue.
	"""
	winname = "Image"
	cv2.namedWindow(winname)        # Create a named window
	cv2.moveWindow(winname, 40,30)  # Move it to (40,30)
	cv2.imshow(winname, img)
	cv2.waitKey()
	cv2.destroyAllWindows()

def cd_sift_ransac(img, template):
	"""
	Implement the cone detection using SIFT + RANSAC algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	# Minimum number of matching features
	MIN_MATCH = 10
	# Create SIFT
	sift = cv2.xfeatures2d.SIFT_create()

	# Compute SIFT on template and test image
	kp1, des1 = sift.detectAndCompute(template,None)
	kp2, des2 = sift.detectAndCompute(img,None)

	# Find matches
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2,k=2)

	# Find and store good matches
	good = []
	for m,n in matches:
		if m.distance < 0.75*n.distance:
			good.append(m)

	# If enough good matches, find bounding box
	if len(good) > MIN_MATCH:
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

		# Create mask
		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
		matchesMask = mask.ravel().tolist()

		h, w = template.shape
		#coordinates of the edges of template img in order of top left, bottom left, bottom right, top right
		pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

		########## YOUR CODE STARTS HERE ##########
		#dst is an array of respective transformed pts by the homography matrix M
                dst = np.int32(cv2.perspectiveTransform(pts, M))

                # get the min and max x and y coordinates of the object
                #x_min = min(dst[x, 0][0] for x in range(4))
                #x_max = max(dst[x, 0][0] for x in range(4))
                #y_min = min(dst[x, 0][1] for x in range(4))
                #y_max = max(dst[x, 0][1] for x in range(4))
		x_min = dst[0,0,0]
		x_max = dst[2,0,0]
		y_min = dst[0,0,1]
		y_max = dst[2,0,1]
		
		#draws the bounding box
                cv2.rectangle(img,(x_min,y_min),(x_max,y_max),(0,255,0),2)
                image_print(img)

		########### YOUR CODE ENDS HERE ###########

		# Return bounding box
		return ((x_min, y_min), (x_max, y_max))
	else:

		print "[SIFT] not enough matches; matches: ", len(good)

		# Return bounding box of area 0 if no match found
		return ((0,0), (0,0))

def cd_template_matching(img, template):
	"""
	Implement the cone detection using template matching algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	template_canny = cv2.Canny(template, 50, 200)
	
	# Perform Canny Edge detection on test image
	grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img_canny = cv2.Canny(grey_img, 50, 200)

	# Get dimensions of template
	(img_height, img_width) = img_canny.shape[:2]

	# Keep track of best-fit match
	#Lisa: as list of [best coeff, bounding box: ([top left coord], [bottom right coord])]
	best_match = [None,None]

	# Loop over different scales of image template
	for scale in np.linspace(1.5, .5, 50):
		# Resize the image
		resized_template = imutils.resize(template_canny, width = int(template_canny.shape[1] * scale))
		(h,w) = resized_template.shape[:2]
		# Check to see if test image is now smaller than template image
		if resized_template.shape[0] > img_height or resized_template.shape[1] > img_width:
			continue
		
		########## YOUR CODE STARTS HERE ##########
		# Use OpenCV template matching functions to find the best match
		# across template scales.
		# Remember to resize the bounding box using the highest scoring scale
		# x1,y1 pixel will be accurate, but x2,y2 needs to be correctly scaled

		# All the 6 methods for comparison in a list
		#['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
            	#	'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']

		#use resized_template and img_canny
		method = eval('cv2.TM_CCORR_NORMED')
		res = cv2.matchTemplate(img_canny,resized_template,method)
		min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
		# If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum for optimal value
		if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
			if best_match == [None, None] or (min_val<best_match[0]):
				top_left = min_loc
				bottom_right = (top_left[0] + w, top_left[1] + h)
				best_match[0] = min_val
				best_match[1] = (top_left, bottom_right)
		else:
			if best_match == [None, None] or (max_val>best_match[0]): #update
				top_left = max_loc
				bottom_right = (top_left[0] + w, top_left[1] + h)
				best_match[0] = max_val
				best_match[1] = (top_left, bottom_right)
	
	bounding_box = best_match[1]#((0,0),(0,0))
		
	cv2.rectangle(img, best_match[1][0], best_match[1][1], 255, 2)
	image_print(img)
	########### YOUR CODE ENDS HERE ###########
	return bounding_box
