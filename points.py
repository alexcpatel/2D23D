#!/usr/bin/env python3.7
import os
import numpy as np
from skimage import io, filters, feature, draw
import matplotlib.pyplot as plt

## SCAN CONSTANTS ##

# taken from blender object properties
CAMERA_POS = np.array([[ 0.00], [7.19], [3.79], [1.00]])
LASER_POS  = np.array([[-2.50], [7.19], [3.79], [1.00]])
TARGET_POS = np.array([[ 0.00], [0.00], [0.50], [1.00]])

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

def pixels_to_screen_points(pixels):
  pass

def detect_laser_pixels(image):
  # extract element-wise channel diff
  image[...,1] >>= 1
  image[...,2] >>= 1
  diff_image = image[...,0] - image[...,1] - image[...,2]

  # compute row-wise highest intensity pixel
  threshold = 100 # this is an arbitrary number
  pixels = []
  for row in range(image.shape[0]):
    coordinates = \
      feature.peak.peak_local_max(diff_image[row,:], threshold_abs=threshold,
                                  num_peaks=3)
    for coordinate in coordinates:
      pixels.append((row,coordinate[0]))
  
  return pixels

def pixels_to_screen_points(pixels):
  pass

image = io.imread('scan/scan00001.png')
pixels = detect_laser_pixels(image)
print(pixels)
