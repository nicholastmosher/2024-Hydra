import cv2
import numpy as np
kernel = np.ones((5,5),np.uint8)
# Define lower and upper bounds for orange color in HSV
LOWER_ORANGE_HSV = np.array([0, 222, 111])
UPPER_ORANGE_HSV = np.array([7, 255, 255])
# The minimum contour area to detect a note
MINIMUM_CONTOUR_AREA = 400
# The threshold for a contour to be considered a disk
CONTOUR_DISK_THRESHOLD = 0.4


# def find_largest_orange_contour(hsv_image: np.ndarray) -> np.ndarray:
#     # Finds the largest orange contour in an HSV image
#     # :param hsv_image: the image to find the contour in
#     # :return: the largest orange contour
    
#     # Threshold the HSV image to get only orange colors
#     mask = cv2.inRange(hsv_image, LOWER_ORANGE_HSV, UPPER_ORANGE_HSV)
#     closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
#     opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
#     # Find contours in the mask
#     contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     if contours:
#         return max(contours, key=cv2.contourArea)


# def contour_is_note(contour: np.ndarray) -> bool:
    
#     # Checks if the contour is shaped like a note
#     # :param contour: the contour to check
#     # :return: True if the contour is shaped like a note
    
#     # Makes sure the contour isn't some random small spec of noise
#     if cv2.contourArea(contour) < MINIMUM_CONTOUR_AREA:
#         return False

#     # Gets the smallest convex polygon that can fit around the contour
#     contour_hull = cv2.convexHull(contour)
#     # Fits an ellipse to the hull, and gets its area
#     ellipse = cv2.fitEllipse(contour_hull)
#     best_fit_ellipse_area = np.pi * (ellipse[1][0] / 2) * (ellipse[1][1] / 2)
#     # Returns True if the hull is almost as big as the ellipse
#     return cv2.contourArea(contour_hull) / best_fit_ellipse_area > CONTOUR_DISK_THRESHOLD

# # runPipeline() is called every frame by Limelight's backend.
# def runPipeline(image, llrobot):
#     blur = cv2.GaussianBlur(image,(5,5),0)
#     # convert the input image to the HSV color space
#     img_hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
#     # convert the hsv to a binary image by removing any pixels 
#     # that do not fall within the following HSV Min/Max values
#     # find contours in the new binary image
#     # contours, _ = cv2.findContours(img_threshold, 
#     # cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     largestContour = find_largest_orange_contour(img_hsv)

#     # initialize an empty array of values to send back to the robot
#     llpython = [0,0,0,0,0,0,0,0]

#     # if contours have been detected, draw them 
#     if largestContour is not None and contour_is_note(largestContour):

#         # # get the unrotated bounding box that surrounds the contour
#         # x,y,w,h = cv2.boundingRect(largestContour)

#         # # draw the unrotated bounding box
#         # cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)

#         # # record some custom data to send back to the robot
#         # llpython = [1,x,y,w,h,9,8,7]  
#         cv2.ellipse(image, cv2.fitEllipse(largestContour), (255, 0, 255), 2)

#     else: largestContour = []
#     #return the largest contour for the LL crosshair, the modified image, and custom robot data
#     return largestContour, img_hsv, llpython

def stage1(image: np.ndarray) -> np.ndarray:
    blur_image = cv2.GaussianBlur(image,(5,5),0)
    hsv_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)
    return hsv_image

def stage2(image: np.ndarray, lower_orange, upper_orange) -> np.ndarray:
    mask = cv2.inRange(image, lower_orange, upper_orange)
    return mask

def stage3(image: np.ndarray) -> np.ndarray:
    closing = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
    opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
    return opening

# def stage4(image: np.ndarray) -> (np.ndarray, np.ndarray):
#     contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     largestContour = []

#     if contours:
#         largestContour = max(contours, key=cv2.contourArea)

#     return image, largestContour

def stage4(image: np.ndarray) -> np.ndarray:
    # Find contours in the mask
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        return max(contours, key=cv2.contourArea)

def is_note(contour: np.ndarray) -> bool:    
    # Makes sure the contour isn't some random small spec of noise
    if cv2.contourArea(contour) < MINIMUM_CONTOUR_AREA:
        return False

    # Gets the smallest convex polygon that can fit around the contour
    contour_hull = cv2.convexHull(contour)

    # Fits an ellipse to the hull, and gets its area
    ellipse = cv2.fitEllipse(contour_hull)
    best_fit_ellipse_area = np.pi * (ellipse[1][0] / 2) * (ellipse[1][1] / 2)

    # Returns True if the hull is almost as big as the ellipse
    return cv2.contourArea(contour_hull) / best_fit_ellipse_area > CONTOUR_DISK_THRESHOLD

def runPipeline(image_input, llrobot):
    llpython = [0,0,0,0,0,0,0,0]

    image1 = stage1(image_input)
    image2 = stage2(image1, LOWER_ORANGE_HSV, UPPER_ORANGE_HSV)
    image3 = stage3(image2)
    largestContour = stage4(image3)
    if largestContour is not None and is_note(largestContour):
        cv2.ellipse(image3, cv2.fitEllipse(largestContour), (255, 0, 255), 2)
    else: largestContour = []

    image_output = image3

    return largestContour, image_input, llpython
