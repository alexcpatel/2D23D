#!/usr/bin/env python3.7
import os
import numpy as np
from skimage import io, filters, feature, draw
import matplotlib.pyplot as plt

## SCAN CONSTANTS (all numbers in meters) ##

# taken from blender object properties
LASER_POS  = np.array([[-2.50], [7.19], [3.79], [1.00]])
TARGET_POS = np.array([[ 0.00], [0.00], [0.50], [1.00]])

CAMERA_POS           = np.array([[ 0.00], [7.19], [3.79], [1.00]])
CAMERA_FOCAL_LENGTH  = 0.050
CAMERA_SENSOR_WIDTH  = 0.036
CAMERA_SENSOR_HEIGHT = 0.024

############################################
## TAKEN FROM BLENDER SCRIPT OF THE FORM: ##
##                                        ##
## import bpy                             ##
## camera = bpy.data.objects['Camera']    ##
## print(camera.matrix_world)             ##
############################################

CAMERA_TO_WORLD = np.linalg.inv(np.array(
  [[-1.0000,  0.0000, 0.0000, 0.0000],
   [ 0.0000, -0.4161, 0.9093, 7.1900],
   [ 0.0000,  0.9093, 0.4161, 3.7900],
   [ 0.0000,  0.0000, 0.0000, 1.0000]]))

# detects pixels along the laser line in the image
# RETURNS: list of laser pixels as (x, y) tuples
def detect_laser_pixels(image, dim):
  # extract element-wise channel diff
  image[...,1] >>= 1; image[...,2] >>= 1
  diff_image = image[...,0] - image[...,1] - image[...,2]

  # compute row-wise highest intensity pixels (max 3 per row)
  threshold = 100 # this is an arbitrary number
  pixels = []
  for row in range(dim[0]):
    coordinates = \
      feature.peak.peak_local_max(diff_image[row,:],
        threshold_abs=threshold, num_peaks=3)
    for coordinate in coordinates:
      pixels.append((coordinate[0], row))
  return pixels

# converts a list of pixels to corresponding screen points
# in world space as a numpy array
# RETURNS: (#pixels)x3 numpy array of screen points
def pixels_to_screen_points(pixels, dim):
  iw, ih  = 1.0 / dim[1], 1.0 / dim[0]
  tx, ty  = iw * CAMERA_SENSOR_WIDTH, ih * CAMERA_SENSOR_HEIGHT
  ox, oy  = CAMERA_SENSOR_WIDTH * 0.5, CAMERA_SENSOR_HEIGHT * 0.5
  npixels = len(pixels)

  # convert pixels to camera space
  camera_points = np.zeros((4, npixels))
  for i in range(npixels):
    x, y = pixels[i][0] * tx - ox, pixels[i][1] * ty - oy
    camera_points[:,i] = np.array([x, y, CAMERA_FOCAL_LENGTH, 1.00])

  # convert pixels to world space
  screen_points = (CAMERA_TO_WORLD @ camera_points)[0:3,:]
  return screen_points

# read scan data
image = io.imread('scan/scan00001.png')
dim   = image.shape

pixels        = detect_laser_pixels(image, dim)
screen_points = pixels_to_screen_points(pixels, dim)
print(dim)
print(pixels[0])
print(screen_points[:,0])
