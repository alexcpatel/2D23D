# 2D23D: An 18-500 CMU ECE Capstone Project by Team B3

## [Website](http://course.ece.cmu.edu/~ece500/projects/s20-teamb3/)
## [Report](http://course.ece.cmu.edu/~ece500/projects/s20-teamb3/wp-content/uploads/sites/85/2020/05/2D23D_final_report-min.pdf)

## Introduction

It cannot be overstated how much traditional 2D camera technology has shaped our
progress and culture. However, 2D images do not effectively capture the details
of an object when it is rotated around. This information is potentially crucial
to archaeological archivists that want to preserve the object in its entirety,
with the potential to 3D print and recreate these objects. Our goal is to be
able to accurately map and scan the physical object into a digital
three-dimensional representation for the purpose of archaeological
documentation. Our design goals are to design this device to be able to be used
by a non-technical audience: easy to use, time-efficient, and most of all,
accurate in scanning. Currently, manual 3D modeling by users is
time-ineffective, tedious, and costly, as well as prone to errors. Our project
will uniquely address these issues in being user-friendly and accurate.

The current iteration of this project successfully implements the software
pipeline required to compliment a completed physical version of the project.
Users can easily attach our developed processing pipeline to data aquired from
physical sensors to perform a working 3D scanning device.

If you are interested in using our code, please investigate the resources
below to understand our project and how to integrate it into your work.

- [A report documenting this projects development](http://course.ece.cmu.edu/~ece500/projects/s20-teamb3/wp-content/uploads/sites/85/2020/05/2D23D_final_report-min.pdf)
- [A wordpress site devoted to this project](http://course.ece.cmu.edu/~ece500/projects/s20-teamb3/)

## Installation

### Requirements
1. Python 3.6.8 with packages:
    - open3d
    - numpy
    - pyvista
    - sklearn
2. Blender 2.8

### Set Up
1. Download or pull this repository.
2. (If performing simulated scans) Collect 3D objects or scenes to be scanned in a local directory.

## Usage

### Acquiring Virtual Scans
Using either ``sim/base.blend`` or ``sim/base_jeremy.blend`` as a reference,
replace the object to be scanned with your custom 3D mesh object. Make sure
that the object is a child of the empty object in the scene that is rotated
by keyframe animations. Set the output settings as appropriate (output
directory, render resolution, number of frames) and render the animation.

Run the script included in ``constants.py`` in blender with the console window
open to gather blender parameters for your rendered animation. Normally these
parameters would need to be calibrated for a physical device, but can be
directly extracted from blender in the case of a simulated scan. If the camera,
laser line, and turntable positions were unchanged from the reference file,
this is not necessary. Overwrite the values of ``CAMERA_TO_WORLD`` and
``LASER_TO_WORLD`` in ``constants.py`` with the results of running the script
in your blender scene.

To perform multiple scans of an object, which may be combined to create a more
accurate 3D reconstruction, simply repeat this process as necessary to acquire
multiple animation render output directories (one for each scan). Note that all
of the scans must have the same constants for ``constants.py``.

Finally, arrange the scans with the following organization structure:
- ``<scan_directory>``
    - ``<scan_0>``
        - ``*00000.png``
        - ``*00001.png``
        - ...
        - ``*n.png``
    - ``<scan_1>``
        - ``*00000.png``
        - ``*00001.png``
        - ...
        - ``*n.png``
    - ...
    - ``<scan_n>``
        - ``*00000.png``
        - ``*00001.png``
        - ...
        - ``*n.png``

The top level ``<scan_directory>`` can be given any name. Each of the
intermediate scan directories (e.g. ``<scan_1>``) can also be given any name.
The names of the images within each scan directory must be labeled in increasing
alphabetical order corresponding to the frame number from the blender rendered
animation.

This ``<scan_directory>`` is the final scan that will be input to the driver
script. Even if you have only performed a single scan for the object, organize
the directories as explained above.

### Running the Software Pipeline

The interface to the software pipeline is the ``scripts/driver.py`` python
script. Usage is below:

```
usage: driver.py [-h] -m MODE [-v] [-d] -s MAIN_DIRECTORY [-o OUT_FILENAME]
                 [-n NUM_ITERS] [-vf VERIFY_FILENAME] [-l LASER_THRESHOLD]
                 [-w WINDOW_LEN] [-p PIXEL_SKIP] [-f IMAGE_SKIP]
                 [-df | -ds | -af | -as | -sp] [-t TRUTH_FILENAME]
                 [-k NUM_KD_TREE_NEIGHBORS] [-i NUM_IPC_POINTS]

optional arguments:
  -h, --help            show this help message and exit
  -m MODE, --mode MODE  <mode (f: full pipeline, p: generate pcds in temp
                        folder, t: start with pcds in directory)>
  -v, --verbose         verbose mode
  -d, --display         display mode
  -s MAIN_DIRECTORY, --input_directory MAIN_DIRECTORY
                        <input_directory>
  -o OUT_FILENAME, --output_filename OUT_FILENAME
                        [output_filename (otherwise <scan_directory>.obj)]
  -n NUM_ITERS, --num_iters NUM_ITERS
                        <num_iters>
  -vf VERIFY_FILENAME, --verify_filename VERIFY_FILENAME
                        [verify_filename (process result into csv)]
  -l LASER_THRESHOLD, --laser_threshold LASER_THRESHOLD
                        [laser_threshold]
  -w WINDOW_LEN, --window_len WINDOW_LEN
                        [window_len]
  -p PIXEL_SKIP, --pixel_skip PIXEL_SKIP
                        [pixel_skip]
  -f IMAGE_SKIP, --image_skip IMAGE_SKIP
                        [image_skip]
  -df, --delaunay_fast  fast delaunay triangulation
  -ds, --delaunay_slow  slow delaunay triangulation
  -af, --alpha_fast     fast alpha shape convex hull triangulation
  -as, --alpha_slow     slow alpha shape convex hull triangulation
  -sp, --poisson        screened poisson triangulation
  -t TRUTH_FILENAME, --ground_truth_filename TRUTH_FILENAME
                        [ground_truth_filename (perform verification)]
  -k NUM_KD_TREE_NEIGHBORS, --num_kd_tree_neighbors NUM_KD_TREE_NEIGHBORS
                        [num_kd_tree_neighbors]
  -i NUM_IPC_POINTS, --num_ipc_points NUM_IPC_POINTS
                        [num_ipc_points]
```

Required arguments are ``-m`` and ``-s``, to indicate the mode of operation
and the source directory. The modes are:

```
-m f     full pipeline: input the scan directory as discussed above
-m p     only generate pcd files (an intermediate representation)
-m t     start operation with known pcd files in the given directory
```

Optional parameters are the following:

```
-v -d             enable verbose and display mode.
-o                specifies an output mesh file name (or output pcd).
                  by default the output name is the same as the input directory.
-n                number of iterations to run the entire pipeline (not supported).
-vf               the ground truth mesh for verification comparison. if supplied,
                  the generated mesh will be compared against the ground truth
                  and verification results will be output in the console by default.
-l -w -p -f       point cloud generation parameters (don't touch unless you know what you're doing)
-df -ds -af -as   triangulation parameters (select one to choose triangulation algorithm).
-t                an output csv file destination to report verification results, if -vf was supplied
-k -i             verification parameters (don't touch unless you know what you're doing)
```

While running the script, a temporary directory ``scripts/temp`` will be
generated containing any intermediate files. These files may be analyzed as
needed to evaluate the performance of the reconstruction.

A simple use case is to set the mode to full pipeline and supply a completed
scan directory. After the script has executed to completion, the output
reconstructed mesh will be placed in the same directory as the scan directory
with the same name (but as an .obj file).

A more in-depth usage overview can be found in our [report](http://course.ece.cmu.edu/~ece500/projects/s20-teamb3/wp-content/uploads/sites/85/2020/05/2D23D_final_report-min.pdf).

### Adapting to Physical Use Case
To adapt this project to a physical device, start with the research we have
conducted in our [report](http://course.ece.cmu.edu/~ece500/projects/s20-teamb3/wp-content/uploads/sites/85/2020/05/2D23D_final_report-min.pdf).

As long as the constants in ``constants.py`` match your static calibration
of the positions of the physical camera and laser, and the position of the center
of the turntable is at ``(0,0,0)`` with ``+Z`` being directly vertical, the software
should work out of the box. However, this calibration procedure can be complex
and is up to the user to develop. The scan directory must be organized in the
same fashion as described above to run the driver script ``scripts/driver.py``.

### Demos
See the reconstructed meshes in the ``obj_out`` directory corresponding to
the meshes in the ``obj_gt`` directory to see examples of the project.

## Contributing
Feel free to fork or clone this repository for any purpose.
Pushing to this repository is not allowed.

## License
MIT License

Copyright (c) 2020 ALEXANDER PATEL and CHAKARA OWARANG and JEREMY LEUNG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
