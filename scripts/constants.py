import numpy as np

DEBUG = True

DEFAULT_LASER_THRESHOLD = 100 # arbitrary
DEFAULT_WINDOW_LEN      = 5   # MUST BE ODD!
DEFAULT_PIXEL_SKIP      = 1

## SCAN CONSTANTS (all units in meters) ##

# taken from blender object properties
CAMERA_FOCAL_LENGTH  = 0.050
CAMERA_SENSOR_WIDTH  = 0.056

"""
USING BLENDER SCRIPT:                                 
                                                                       
for object_name in ["Camera", "Laser"]:                                
  object = bpy.data.objects[object_name]                             
  print("# " + object_name + "\n[", end="")                          
  for i in range(4):                                                 
    print("[" if i == 0 else " [", end="")                         
    for j in range(4):                                             
      print(decimal.Decimal(object.matrix_world[i][j]), end="")  
      print("" if j == 3 else ", ", end="")                      
    print("]" if i == 3 else "],\n", end="")                       
  print("]", end="\n")
"""

CAMERA_TO_WORLD = np.array( \
[[-0.999999940395355224609375, 0, 0, 0],
 [0, -0.19126594066619873046875, 0.981538295745849609375, 6.1840057373046875],
 [0, 0.981538236141204833984375, 0.1912659704685211181640625, 2.2050368785858154296875],
 [0, 0, 0, 1]])

LASER_TO_WORLD  = np.array( \
[[-0.000881829299032688140869140625, -0.07986216247081756591796875, 0.4647571742534637451171875, 3.306972026824951171875],
 [0.00047156887012533843517303466796875, -0.14934146404266357421875, 0.869091451168060302734375, 6.1840057373046875],
 [-7.450580950807417224268647260032594203948974609375E-12, 0.985555231571197509765625, 0.16935418546199798583984375, 2.2050368785858154296875],
 [0, 0, 0, 1]])

## GLOBAL VARIABLES ##

def norm(v): return v / np.sqrt(np.sum(v**2))

CAMERA_POS = (CAMERA_TO_WORLD @ np.array([[0.0],[0.0],[0.0],[1.0]]))[0:3,:][:,0]
LASER_POS  = (LASER_TO_WORLD @ np.array([[0.0],[0.0],[0.0],[1.0]]))[0:3,:][:,0]

# +X is perpendicular to laser line in laser space,
# this transforms that vector into world space to get
# the laser plane normal
LASER_N = norm((LASER_TO_WORLD @ \
  np.array([[-1.0],[0.0],[0.0],[1.0]]))[0:3,:][:,0] - LASER_POS)

LASER_P = LASER_N * np.dot(LASER_N, LASER_POS)
