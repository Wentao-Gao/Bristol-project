import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
from scipy import ndimage
from skimage.color import rgb2hsv
import argparse


# Read in images
image = cv2.imread('/Users/y/Documents/Python_code/Apples.jpg')
colour = image.copy()

# Build a parameter parser
parser = argparse.ArgumentParser()
parser.add_argument("-i","--image", required=True, help="Path to input image")
args = vars(parser.parse_args())

# Load the apple image
img = cv2.imread(args['image'])

# Convert to HSV
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
img_1 = img.copy

# Red range in HSV space
low_apple_red_1 = (156.0, 155.0, 155.0)
high_apple_red_1 = (180.0, 255.0, 255.0)
low_apple_red_2 = (0.0, 155.0, 155.0)
high_apple_red_2 = (10.0, 255.0, 255.0)

# Build a mask image
mask_red_1 = cv2.inRange(hsv_img,low_apple_red_1, high_apple_red_1)
mask_red_2 = cv2.inRange(hsv_img,low_apple_red_2, high_apple_red_2)
mask = mask_red_1 + mask_red_2

"""Morphological operations for separating apples"""

# create a structuring element
kernel = np.ones((3,3),np.uint8)
# closing
closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
# erosion
kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(7,7))
erosion = cv2.erode(closing,kernel,iterations = 7)
# closing
kernel = np.ones((11,11),np.uint8)
closing = cv2.morphologyEx(erosion, cv2.MORPH_CLOSE, kernel, iterations=8)
# erosion
kernel = np.ones((5,5),np.uint8)
erosion2 = cv2.erode(closing,kernel,iterations = 6)
# remove boarder pixels
erosion2[:50, :] = 0
erosion2[:, :50] = 0
erosion2[-50:, :] = 0
erosion2[:, -50:] = 0

"""Find centroids of isolated clusters/blobs"""

display = colour.copy()
labels, nlabels = ndimage.label(erosion2)  # Label features in an array. Any non-zero values in input are counted as features and zero values are considered the background.
print("There are " + str(nlabels) + " apples")

centroid = ndimage.center_of_mass(erosion2, labels, np.arange(nlabels) + 1 ) # calculate the center of mass of the values of an array at labels.

# draw circles representing the centroids
for cen in centroid:
  display = cv2.circle(display, (cen[1].astype(int), cen[0].astype(int)), radius=25, color=(0, 0, 0), thickness=-3)

plt.figure()
plt.imshow(display[:,:,::-1])