#!/usr/bin/env python3.7
import math
import open3d as o3d
import numpy as np
from skimage import io, feature

def norm(v): return v / np.sqrt(np.sum(v**2))

## SCAN CONSTANTS (all units in meters) ##
NUM_IMAGES = 100

# taken from blender object properties
LASER_POS  = np.array([-2.50, 7.19, 3.79])
TARGET_POS = np.array([ 0.00, 0.00, 0.50])
CAMERA_POS = np.array([ 0.00, 7.19, 3.79])

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

CAMERA_TO_WORLD = np.array( \
  [[-1.0000,  0.0000, 0.0000, 0.0000],
   [ 0.0000, -0.4161, 0.9093, 7.1900],
   [ 0.0000,  0.9093, 0.4161, 3.7900],
   [ 0.0000,  0.0000, 0.0000, 1.0000]])

LASER_TO_WORLD  = np.array( \
  [[-0.7342,  0.0400, -0.0956, -2.5000],
   [-0.2553, -0.1151,  0.2751,  7.1900],
   [ 0.0000,  0.2821,  0.1259,  3.7900],
   [ 0.0000,  0.0000,  0.0000,  1.0000]])

## GLOBAL VARIABLES ##

# +X is perpendicular to laser line in laser space,
# this transforms that vector into world space to get
# the laser plane normal
LASER_N = norm((LASER_TO_WORLD @ \
  np.array([[1.0],[0.0],[0.0],[1.0]]))[0:3,:][:,0] - LASER_POS)

# angle of rotation between each successive scan image
ROT_ANGLE = (2.0 * math.pi) / NUM_IMAGES

## HELPER FUNCTIONS ##

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
# RETURNS: list of screen points
def pixels_to_screen_points(pixels, dim):
  iw, ih  = 1.0 / dim[1], 1.0 / dim[0]
  tx, ty  = iw * CAMERA_SENSOR_WIDTH, ih * CAMERA_SENSOR_HEIGHT
  ox, oy  = CAMERA_SENSOR_WIDTH * 0.5, CAMERA_SENSOR_HEIGHT * 0.5
  npixels = len(pixels)

  # convert pixels to camera space
  camera_points = np.zeros((4, npixels))
  for i in range(npixels):
    x, y = pixels[i][0] * tx - ox, pixels[i][1] * ty - oy
    camera_points[:,i] = np.array([x, -y, -CAMERA_FOCAL_LENGTH, 1.00])

  # convert pixels to world space
  screen_points = (CAMERA_TO_WORLD @ camera_points)[0:3,:]
  return np.ndarray.tolist(screen_points.T)

# Perform ray plane intersection from the camera origin through
# each screen point to the laser plane to determine the point of
# the laser on the object in world space. This function removes
# points relating to the background by checking the Z coordinate.
# RETURNS: list of world points
def screen_points_to_laser_plane(screen_points):
  world_points = []
  numerator = -np.dot(CAMERA_POS, LASER_N)
  for point in screen_points:
    raydir = norm(point - CAMERA_POS)
    denom  = np.dot(LASER_N, raydir)
    if abs(denom) < 1e-6: continue
    time   = numerator / denom
    world  = CAMERA_POS + raydir * time
    if time < 0 or time > CAMERA_POS[1] + 1.0 \
       or world[2] < 0: continue
    world_points.append(world)
  return world_points

# Reverse rotate points along the center axis based on
# image index. This gets the corresponding point on the object.
# RETURNS: list of object points
def world_points_to_object_points(world_points, i):
  angle = -ROT_ANGLE * (i-1)
  cosine, sine = math.cos(angle), math.sin(angle)
  rotate = np.array( \
    [[cosine,  -sine, 0.0],
     [  sine, cosine, 0.0],
     [   0.0,    0.0, 1.0]])
  object_points = []
  for world_point in world_points:
    object_points.append( \
      np.ndarray.flatten((rotate @ world_point.reshape((-1, 1)))))
  return object_points

## POINT CLOUD GENERATION SCRIPT ##

# generate all points for point cloud from scan
points = []
for i in range(1, NUM_IMAGES+1):
  print("processing image %d..." % i)

  # read a scan image
  image = io.imread('scan/scan%05d.png' % i)
  dim   = image.shape

  # generate points for that image and add to point cloud
  pixels        = detect_laser_pixels(image, dim)
  screen_points = pixels_to_screen_points(pixels, dim)
  world_points  = screen_points_to_laser_plane(screen_points)
  object_points = world_points_to_object_points(world_points, i)
  points.extend(object_points)

print("%d points generated" % len(points))
print("converting points to numpy array...")
np_points = np.asarray(points, dtype=np.float64)
pcl = o3d.geometry.PointCloud()
print("formatting pcd file...")
pcl.points = o3d.utility.Vector3dVector(np_points)
print("writing pcd file...")
o3d.io.write_point_cloud("point_cloud.pcd", pcl)

## Triangulating Mesh ##

import pyvista as pv

cloud  = pv.PolyData(np_points)
volume = cloud.delaunay_3d(tol=0.5)
shell  = volume.extract_geometry()
shell.plot()
shell.save("mesh.stl")
