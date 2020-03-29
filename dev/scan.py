#!/usr/bin/env python3
import os
import numpy as np
from skimage import io, filters, feature, draw
import matplotlib.pyplot as plt

def detect_laser_points(image):
  # extract element-wise channel diff
  h, w = image.shape[0], image.shape[1]
  diff_image = np.array(np.zeros((h, w)), dtype='uint8')
  for i in range(h):
    for j in range(w):
      diff_image[i,j] = max(0,
        image[i,j,0] - image[i,j,1] // 2 - image[i,j,2] // 2)

  # compute row-wise highest intensity channel
  threshold = 100 # this is an arbitrary number
  points = []
  for row in range(h):
    coordinates = \
      feature.peak.peak_local_max(diff_image[row,:], threshold_abs=threshold)
    if len(coordinates) > 0:
      for coordinate in coordinates:
        points.append((row,coordinate[0]))

  # add channels back to diff image
  point_image = np.array(np.zeros((h, w, 3)), dtype='uint8')
  for i in range(h):
    for j in range(w):
      diff = diff_image[i,j]
      point_image[i,j,0] = diff
      point_image[i,j,1] = diff
      point_image[i,j,2] = diff

  # draw points as red circles on point_image
  for point in points:
    r, c = point
    rr, cc = draw.circle_perimeter(r, c, 3)
    _rr, _cc = [], []
    for idx in range(len(rr)):
      if rr[idx] >= 0 and rr[idx] < h and \
         cc[idx] >= 0 and cc[idx] < w:
         _rr.append(rr[idx])
         _cc.append(cc[idx])
    rr, cc = _rr, _cc
    point_image[rr,cc] = np.array([255,0,0], dtype='uint8')

  return diff_image, point_image

image = io.imread('laser_detect.png')
diff_image, point_image = detect_laser_points(image)
io.imsave('diff_image.png', diff_image)
io.imsave('point_image.png', point_image)
