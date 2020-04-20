import os, sys, math, time
import numpy as np
import open3d as o3d
from skimage import io
from constants import *

## HELPER FUNCTIONS ##

# detects pixels along the laser line in the image
# RETURNS: list of laser pixels as (x, y) tuples
def detect_laser_pixels(image):
  # extract element-wise channel diff intensity
  image[...,1] >>= 1; image[...,2] >>= 1
  intensity = image[...,0] - image[...,1] - image[...,2]
  rows, cols = intensity.shape

  # apply hamming window to smooth intensity rows
  window      = np.hamming(WINDOW_LEN)
  intensity   = np.reshape(intensity, (rows*cols,))
  filtered    = np.convolve(intensity, window, mode='same')

  # compute row-wise local max intensity pixels
  mask        = np.zeros((rows*cols,), dtype=np.bool)
  mask[1:-1]  = np.diff(np.sign(np.diff(filtered))) < 0
  mask       &= intensity > LASER_THRESHOLD
  mask        = np.reshape(mask, (rows,cols))

  return np.ndarray.tolist(np.argwhere(mask))

# converts a list of pixels to corresponding screen points
# in world space as a numpy array
# RETURNS: list of screen points
def pixels_to_screen_points(pixels, dim):
  camera_sensor_height = CAMERA_SENSOR_WIDTH * (dim[0] / dim[1])
  iw, ih  = 1.0 / dim[1], 1.0 / dim[0]

  # convert pixels to camera space
  camera_points = np.zeros((4, len(pixels)))
  for i in range(len(pixels)):
    px, py = pixels[i][1] + 0.5, pixels[i][0] * PIXEL_SKIP + 0.5
    nx, ny = px * iw - 0.5, py * ih - 0.5
    cx, cy = nx * CAMERA_SENSOR_WIDTH, ny * camera_sensor_height
    camera_points[:,i] = np.array([cx, -cy, -CAMERA_FOCAL_LENGTH, 1.00])

  # convert pixels to world space
  screen_points = (CAMERA_TO_WORLD @ camera_points)[0:3,:].T
  return np.ndarray.tolist(screen_points)

# Perform ray plane intersection from the camera origin through
# each screen point to the laser plane to determine the point of
# the laser on the object in world space. This function removes
# points relating to the background by checking the Z coordinate.
# RETURNS: list of world points
def screen_points_to_laser_plane(screen_points):
  normal, center, origin = LASER_N, LASER_P, CAMERA_POS
  numerator = np.dot(center - origin, normal)
  
  world_points = []
  for point in screen_points:
    direction = norm(point - CAMERA_POS)
    denom     = np.dot(normal, direction)
    if abs(denom) < 1.0e-6: continue
    time      = numerator / denom
    world     = CAMERA_POS + direction * time
    if time < 0 or world[2] < 0: continue
    world_points.append(world)

  return world_points

# Reverse rotate points along the center axis based on
# image index. This gets the corresponding point on the object.
# RETURNS: list of object points
def world_points_to_object_points(world_points, angle):
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
def generate_points(scan_dir):
  _, _, image_names = next(os.walk(scan_dir), (None, None, []))
  image_num = len(image_names)
  base_angle = (2.0 * math.pi) / image_num

  time_pixels        = 0
  time_screen_points = 0
  time_world_points  = 0
  time_object_points = 0

  points = []
  for i in range(image_num):
    image_name = os.path.join(scan_dir, image_names[i])
    print("processing image %d, file %s..." % (i, image_name))

    # read a scan image
    image = io.imread(image_name)
    dim   = image.shape
    image = image[::PIXEL_SKIP,...]
    angle = -base_angle * i

    # generate points for that image and add to point cloud
    start                = time.time()
    pixels               = detect_laser_pixels(image)
    time_pixels         += time.time() - start

    start                = time.time()
    screen_points        = pixels_to_screen_points(pixels, dim)
    time_screen_points  += time.time() - start

    start                = time.time()
    world_points         = screen_points_to_laser_plane(screen_points)
    time_world_points   += time.time() - start

    start                = time.time()
    object_points        = world_points_to_object_points(world_points, angle)
    time_object_points  += time.time() - start

    points.extend(object_points)

  print("time_pixels        =", time_pixels)
  print("time_screen_points =", time_screen_points)
  print("time_world_points  =", time_world_points)
  print("time_object_points =", time_object_points)

  return points

def add_debugging_visualizations(points):
  # add camera position to point cloud
  points.append(CAMERA_POS)

  # add laser plane normal to point cloud
  step = 5.0 / 100.0
  ray = LASER_N
  for i in range(100):
    time = i * step
    points.append(LASER_P + ray * time)

  # add laser plane grid to point cloud
  step1 = 5.0 / 10.0
  ray1 = norm(LASER_POS - LASER_N * np.dot(LASER_POS, LASER_N))
  ray2 = norm(np.cross(LASER_N, ray1))
  for i in range(10):
    for j in range(10):
      x, y = j * step1 - 2.5, i * step1 - 2.5
      points.append(LASER_P + x * ray1 + y * ray2)

  # add xyz axes to point cloud
  stepxyz = 5.0 / 20.0
  rayx = np.array([1.0, 0.0, 0.0])
  rayy = np.array([0.0, 1.0, 0.0])
  rayz = np.array([0.0, 0.0, 1.0])
  for i in range(20):
    amt = i * stepxyz - 2.5
    points.append(amt * rayx)
    points.append(amt * rayy)
    points.append(amt * rayz)

  return points

def main():
  if len(sys.argv) != 2:
    print("Usage: python points.py <scan_dir>")
    exit(-1)
  scan_dir = sys.argv[1]
  if not os.path.isdir(scan_dir):
    print("Error: scan_dir argument (%s) is not a valid directory" % scan_dir)
    exit(-1)
  if len(os.listdir(scan_dir)) == 0:
    print("Error: scan_dir argument (%s) does not contain any files" % scan_dir)
    exit(-1)
  print("Using image scan directory " + scan_dir)

  points = generate_points(scan_dir)
  if DEBUG:
    points = add_debugging_visualizations(points)

  print("%d points generated" % len(points))
  print("converting points to numpy array...")
  np_points = np.asarray(points, dtype=np.float64)
  pcl = o3d.geometry.PointCloud()
  print("formatting pcd file...")
  pcl.points = o3d.utility.Vector3dVector(np_points)
  print("writing pcd file...")
  o3d.io.write_point_cloud("monkey_misalign_02x.pcd", pcl)

  pcd = o3d.io.read_point_cloud("monkey_misalign_02x.pcd")
  o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
  main()
