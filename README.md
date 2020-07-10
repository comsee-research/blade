BLADE: Blur Aware Depth Estimation with a plenoptic camera
=====================================================

BLADE is a set of tools to estimate depth map from raw images obtained by a multifocus plenoptic camera (e.g., a Raytrix R12) based on the [libpleno].


Quick Start
===========

### Pre-requisites

The BLADE applications have a light dependency list:

 * [boost] version 1.54 and up, portable C++ source libraries,
 * [libpleno], an open-souce C++ library for plenoptic camera,
 
and was compiled on:
 * Ubuntu 18.04.4 LTS.
  
### Compilation & Test

If you are comfortable with Linux and CMake and have already installed the prerequisites above, the following commands should compile the applications on your system.

```
mkdir build && cd build
cmake ..
make -j6
```

To test the `depth` application you can use the example script from the build directory:
```
./../example/depth.sh
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
./depth -i images.js -c camera.js -p params.js -o depth.png -v true -g true -l 7
```
  
Datasets
========

Datasets R12-A, R12-B and R12-C can be downloaded [from here](https://github.com/comsee-research/plenoptic-datasets).

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
