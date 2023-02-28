BLADE: Blur Aware Depth Estimation with a plenoptic camera
=====================================================

BLADE is a set of tools to estimate depth map from raw images obtained by (multifocus) plenoptic cameras (e.g., a Raytrix R12) based on the [libpleno].

Quick Start
===========

### Pre-requisites

The BLADE applications have a light dependency list:

 * [boost] version >1.54 and up, portable C++ source libraries,
 * [libpleno], an open-souce C++ library for plenoptic camera,
 
and was compiled and tested on:
  * Ubuntu 18.04.4 LTS, GCC 7.5.0, with Eigen 3.3.4, Boost 1.65.1, and OpenCV 3.2.0,
  * Ubuntu 20.04.5 LTS, GCC 9.4.0, with Eigen 3.3.7, Boost 1.71.0, and OpenCV 4.2.0.
  
### Compilation & Test

If you are comfortable with Linux and CMake and have already installed the prerequisites above, the following commands should compile the applications on your system.

```
mkdir build && cd build
cmake ..
make -j6
```

To test the `depth` application you can use the example script from the build directory:
```
./../examples/depth.sh
```

Applications
============

### Configuration

All applications use _.js (json)_ configuration file. The path to this configuration files are given in the command line using _boost program options_ interface.

**Options:**

| short 	| long 			| default 			| description 								|
|-------	|------			|:---------:			|:-----------:								|
| -h 		| -\-help  		|           		| Print help messages						|
| -g 		| -\-gui  		| `true`          	| Enable GUI (image viewers, etc.)			|
| -v 		| -\-verbose 	| `true`          	| Enable output with extra information		|
| -l 		| -\-level  	| `ALL` (15)       	| Select level of output to print (can be combined): NONE=0, ERR=1, WARN=2, INFO=4, DEBUG=8, ALL=15 |
| -i 		| -\-pimages 	|                	| Path to images configuration file |
| -c 		| -\-pcamera 	|                	| Path to camera configuration file |
| -p 		| -\-pparams 	| `"internals.js"` 	| Path to camera internal parameters configuration file |
| -o 		| -\-output  	| `"depth.png"`		| Path to save estimated depth map |

For instance to run depth estimation:
```
./src/depth/depth -i images.js -c camera.js -p params.js -o depth.png -v true -g true -l 7
```

Five applications are included in BLADE.

### Depth estimation
`depth` runs depth estimations on input images according to the selected strategy.

`depth_from_obs` runs depth estimations on input images according to the selected strategy at micro-images containing BAP features only.
	
**Requirements:** image(s), camera parameters, internal parameters, strategy configuration.

**Output:** raw depth map(s), point cloud(s), central sub-aperture depth map(s).
	
### Depth scaling calibration
`scaling` runs the depth scaling calibration process.
	
**Requirements:** images, camera parameters, internal parameters, scene configuration, raw depth maps, features.

**Output:** camera parameters, scale error statistics (`.csv`).

### Relative depth evaluation	
`evaluate` runs the evaluations of relative depth estimation with respect to a ground truth. Supported depth formats include: raw depth maps, point clouds, `.csv`, `.pts`, `.xyz`, poses, `.mat` and planes.
	
**Requirements:** camera parameters, internal parameters, ground truth and depth information.

**Output:** absolute and relative errors statistics (`.csv`).

### Extrinsic calibration lidar-camera
`lidarcamera` runs the extrinsic parameters calibration from LIDAR frame to camera frame, and graphically check the point clouds.
	
**Requirements:** camera parameters, internal parameters, calibration image, constellation configuration.

**Output:** extrinsic parameters.

### Absolute depth evaluation	
`distances` evaluates distances between reference point cloud and computed depth information, either, directly from central sub-aperture depth map(s) or point cloud(s) or raw depth map(s).
	
**Requirements:** camera parameters, internal parameters, extrinsic lidar-camera parameters, images, reference point cloud (`.pts`), depth information to evaluate.

**Output:** error maps, distances.

  
Datasets
========

* Datasets R12-A, R12-B and R12-C can be downloaded [from here](https://github.com/comsee-research/plenoptic-datasets).
* The dataset R12-D, and the simulated _unfocused plenoptic camera_ dataset UPC-S are also available [from here](https://github.com/comsee-research/plenoptic-datasets).
* Datasets R12-E, ES and ELP20 are available [here](https://github.com/comsee-research/plenoptic-datasets).

Citing
======

If you use BLADE or [libpleno] in an academic context, please cite the following publication:

	@inproceedings{labussiere2020blur,
	  title 	=	{Blur Aware Calibration of Multi-Focus Plenoptic Camera},
	  author	=	{Labussi{\`e}re, Mathieu and Teuli{\`e}re, C{\'e}line and Bernardin, Fr{\'e}d{\'e}ric and Ait-Aider, Omar},
	  booktitle	=	{Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
	  pages		=	{2545--2554},
	  year		=	{2020}
	}
	
or 

	@article{labussiere2022calibration
	  title		=	{Leveraging blur information for plenoptic camera calibration},
	  author	=	{Labussi{\`{e}}re, Mathieu and Teuli{\`{e}}re, C{\'{e}}line and Bernardin, Fr{\'{e}}d{\'{e}}ric and Ait-Aider, Omar},
	  doi		=	{10.1007/s11263-022-01582-z},
	  journal	=	{International Journal of Computer Vision},
	  year		=	{2022},
	  month		=	{may},
	  number	=	{2012},
	  pages		=	{1--23}
	}



License
=======

BLADE is licensed under the GNU General Public License v3.0. Enjoy!

[Ubuntu]: http://www.ubuntu.com
[CMake]: http://www.cmake.org
[CMake documentation]: http://www.cmake.org/cmake/help/cmake2.6docs.html
[git]: http://git-scm.com
[Eigen]: http://eigen.tuxfamily.org
[libv]: http://gitlab.ip.uca.fr/libv/libv
[lma]: http://gitlab.ip.uca.fr/libv/lma
[OpenCV]: https://opencv.org/
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/
[boost]: http://www.boost.org/
[libpleno]: https://github.com/comsee-research/libpleno

---
