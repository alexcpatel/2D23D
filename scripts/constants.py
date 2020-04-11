import numpy as np

## SCAN CONSTANTS (all units in meters) ##

# taken from blender object properties
CAMERA_FOCAL_LENGTH  = 0.050
CAMERA_SENSOR_WIDTH  = 0.036
CAMERA_SENSOR_HEIGHT = 0.024

#############################################################################
##  TAKEN FROM BLENDER SCRIPT OF THE FORM:                                 ##
##                                                                         ##
##  for object_name in ["Camera", "Laser"]:                                ##
##      object = bpy.data.objects[object_name]                             ##
##      print("# " + object_name + "\n[", end="")                          ##
##      for i in range(4):                                                 ##
##          print("[" if i == 0 else " [", end="")                         ##
##          for j in range(4):                                             ##
##              print(decimal.Decimal(object.matrix_world[i][j]), end="")  ##
##              print("" if j == 3 else ", ", end="")                      ##
##          print("]" if i == 3 else "],\n", end="")                       ##
##      print("]", end="\n")                                               ##
#############################################################################

CAMERA_TO_WORLD = np.array( \
[[-0.999999940395355224609375, 0, 0, 0],
 [0, -0.37870180606842041015625, 0.9255187511444091796875, 8.45724964141845703125],
 [0, 0.92551863193511962890625, 0.3787018358707427978515625, 4.4605197906494140625],
 [0, 0, 0, 1]])

LASER_TO_WORLD  = np.array( \
[[-0.00095020234584808349609375, -0.112928591668605804443359375, 0.2904528677463531494140625, 2.7736899852752685546875],
 [0.00031163400853984057903289794921875, -0.344330251216888427734375, 0.885618984699249267578125, 8.45724964141845703125],
 [0, 0.932032167911529541015625, 0.3623757064342498779296875, 4.4605197906494140625],
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
