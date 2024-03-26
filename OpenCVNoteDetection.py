import cv2
import numpy as np
kernel = np.ones((5,5),np.uint8)
# Define lower and upper bounds for orange color in HSV
LOWER_ORANGE_HSV = np.array([0, 150, 111])
UPPER_ORANGE_HSV = np.array([2, 255, 255])
# The minimum contour area to detect a note
MINIMUM_CONTOUR_AREA = 200
# The threshold for a contour to be considered a disk
CONTOUR_DISK_THRESHOLD = 0.35


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
