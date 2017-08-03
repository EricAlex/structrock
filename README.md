# Structrock

A cross-platform outcrop point cloud processing system.

Structrock is released under the terms of the BSD license, and thus free for commercial and research use.
Feel free to analyze your own outcrop point cloud data, to add Structrock into your own project.
You are welcomed to make contributions to this project. Email ericrussell@zju.edu.cn, or open an issue here on GitHub for support.

Citation (more to be added soon):

``` tex
@article{Wang17,
	title={A region-growing approach for automatic outcrop fracture extraction from a three-dimensional point cloud},
	author={Wang, Xin and Zou, Lejun and Shen, Xiaohua and Ren, Yupeng and Qin, Yi},
	journal={Computers \& Geosciences},
	volume={99},
	pages={100-106},
	year={2017},
	doi={10.1016/j.cageo.2016.11.002},
	publisher={Elsevier},
}
@article{Wang_arxiv2017,
  title={Outcrop fracture characterization on suppositional planes cutting through digital outcrop models (DOMs)},
  author={Wang, Xin and Zou, Lejun and Ren, Yupeng and Qin, Yi and Guo, Zhonghao and Shen, Xiaohua},
  journal={arXiv preprint arXiv:1707.03437},
  year={2017}
}
```

![Detailed results.](https://github.com/EricAlex/structrock/blob/master/wiki/img/detailed_results.jpg)
*Detailed results obtained from a portion of the point cloud. (a) A portion of the outcrop and (b) its point cloud. (c) Segmentation results obtained using the proposed algorithm. Different fracture regions are shown in various colors, and the non-fracture regions are shown in red. (d) Estimated fracture planes obtained from the segmentation results.*

![Outcrop results.](https://github.com/EricAlex/structrock/blob/master/wiki/img/outcrop_results.jpg)
*(a) The outcrop used to test the proposed algorithm and (b) the segmentation results. Different fracture regions are shown by different colors, and the non-fracture regions are shown in red.*

## Dependencies

### Point Cloud Library (PCL)

PCL is a standalone, large scale, open project for 2D/3D image and point cloud processing. Official website: http://pointclouds.org/.
Github repository: https://github.com/PointCloudLibrary/pcl.

Version required: 1.6, still having compatibility problems with versions > 1.6.

### Qt

Qt is a cross-platform C++ application development framework. Official website: http://www.qt.io/.

Version required: >= 4.8.

### PostgreSQL

PostgreSQL often simply Postgres, 
is an object-relational database management system (ORDBMS) with an emphasis on extensibility and standards-compliance.
It is used to support an experimental function of structrock: accessing very large LiDAR datasets stored in PostgreSQL databases.

## Functions

Structrock provide a platform on which workflows of processing outcrop point clouds can be designed. A region-growing algorithm for automatic outcrop fracture extraction from a three-dimensional point cloud is implemented.

Structrock can run in both GUI and command-line modes.
In GUI mode, users interact with Structrock to perform certain processing action and provide processing parameters through input dialogs.
In command-line mode, Structrock receive the path to a text file (we call it the workflow file) that contains processing actions and parameters. Lines of actions are separated by ";" and parameters are separated by ",".
It will do these processing actions one by one until the end or errors occur.

An example of workflow file:

``` txt
openpcd,E:\Downloads\parameters_test\0134067-Data-ready.pcd,mute;
knnormal,30,mute;
rgsegmentation,9,0.06,20,4,30,false,mute;
saveclusters,E:\Downloads\parameters_test\Clusters\Data_9_20.bin,mute;
quitsession;
``` 

## Build from source

First, all dependencies, PCL(1.6), Qt, boost, Eigen, FLANN, VTK, QHull and OpenNI should be installed properly.

Download and install CMake, Official website: https://cmake.org/.

Make a new folder named "build" beside the folder containing source files (call it, say, "src").

### On Windows

Make sure Visual Studio 2010 is properly installed.
Open CMake, set "Where is the source code" to the path to "src" and set "Where to build the binary" to the path to "build".
Click "Configure", and select Visual Studio 10 2010, if no errors occur, click "Generate".
Open generated "structrock.sln" file, set "structrock" as the "StartUp Project",
add "/ENTRY:mainCRTStartup" to "structrock"'s "Linker->Command Line" properties, start compilation.

### On Linux

Open terminal window, go to "build" folder, run "cmake ../src", then run "make".

### After a successful compilation, Structrock looks like this:

![GUI.](https://github.com/EricAlex/structrock/blob/master/wiki/img/gui.jpg)

## Processing actions

### openpcd

Description: Open a .pcd file.

Parameters:

1. "The path and file name of the file to open" (required),

2. mute (optional, follow required parameters, if declared, the result of this action will not be displayed on the screen),

3. nolog (optional, follow required parameters, if declared, this action and the time it takes to perform this action will not be logged)

### openxyz

Description: Open a .txt file that contains 3 columns: x, y, z.

Parameters:

1. "The path and file name of the file to open" (required),

2. mute (optional, follow required parameters, if declared, the result of this action will not be displayed on the screen),

3. nolog (optional, follow required parameters, if declared, this action and the time it takes to perform this action will not be logged)

### savepcdascii

Description: Save current data to .pcd file with ASCII encoding.

Parameters:

1. "The path and file name of the file to save" (required)

### savepcdbinary

Description: Save current data to .pcd file with binary encoding.

Parameters:

1. "The path and file name of the file to save" (required)

### savenormals

Description: Save current normals to .pcd file.

Parameters:

1. "The path and file name of the file to save" (required)

### saveclusters

Description: Perform post-outcrop-fracture-extraction actions and save the results into .bin and txt files.

Parameters:

1. "The path and file name of the file to save" (required),

2. mute (optional, follow required parameters, if declared, the result of this action will not be displayed on the screen),

3. nolog (optional, follow required parameters, if declared, this action and the time it takes to perform this action will not be logged)

### downsample

Description: downsample the point cloud given the minimum point distance (meter).

Parameters:

1. "Minimum point distance" (required, double),

2. mute (optional, follow required parameters, if declared, the result of this action will not be displayed on the screen),

3. nolog (optional, follow required parameters, if declared, this action and the time it takes to perform this action will not be logged)

### resample

Description: resample the point cloud given the search radius (meter).

Parameters:

1. "Search radius" (required, double),

2. mute (optional, follow required parameters, if declared, the result of this action will not be displayed on the screen),

3. nolog (optional, follow required parameters, if declared, this action and the time it takes to perform this action will not be logged)

### knnormal

Description: find k nearest neighbors given k, and calculate the normal of the neighbors.

Parameters:

1. "Number of neoghbor points" (required, int),

2. showcurvature (optional, follow required parameters, but before "mute" and "nolog", if declared, the curvature map on the point cloud will be displayed on the screen),

3. mute (optional, follow required parameters, if declared, the result of this action will not be displayed on the screen),

4. nolog (optional, follow required parameters, if declared, this action and the time it takes to perform this action will not be logged)

### ranormal

Description: find the neighbors given the search radius, and calculate the normal of the neighbors.

Parameters:

1. "Search radius" (required, double),

2. showcurvature (optional, follow required parameters, but before "mute" and "nolog", if declared, the curvature map on the point cloud will be displayed on the screen),

3. mute (optional, follow required parameters, if declared, the result of this action will not be displayed on the screen),

4. nolog (optional, follow required parameters, if declared, this action and the time it takes to perform this action will not be logged)

### rostatic

Description: statistically remove the outliers of the point cloud given the standard deviation.

Parameters:

1. "Standard deviation" (required, double),

2. mute (optional, follow required parameters, if declared, the result of this action will not be displayed on the screen),

3. nolog (optional, follow required parameters, if declared, this action and the time it takes to perform this action will not be logged)

### rgsegmentation

Description: perform the Region Growing Segmentation given the parameters.

Parameters:

1. "Local surface normal deviation threshold" (required, double, degree),

2. "Curvature threshold" (required, double, only used when the smooth mode is on),

3. "Transmission error threshold" (required, double, degree, only used when the smooth mode is off),

4. "Minimum number of point a segmented patch must have to be recorded" (required, int),

5. "The number of neighbors will participate in the region growing" (required, int),

6. "Is it on smooth mode?" (required, boolean),

7. mute (optional, follow required parameters, if declared, the result of this action will not be displayed on the screen),

8. nolog (optional, follow required parameters, if declared, this action and the time it takes to perform this action will not be logged)

### showsfeature

Description: Show different kinds of features on the fracture faces or on the outcrop. Features include the fracture faces' roughness, area, linear features, circular features, wave-like features, and etc.

Parameters:

1. "The name of the feature" (required, currently "roughness" and "area" are included.)

2. "Percentage Out threshold" (Optional; Less than 0.5 and bigger than 0.0. If provided, drop the head and the tail in the distribution of the feature to show the variation of the majority of the feature data.)

### quitsession

Description: quit Structrock.

Parameters:

No parameters.

### showstereonet

Description: Show and save (as PDF) the stereonet plot of the fracture data.

Parameters:

1. "The path and file name of the file to save" (required)