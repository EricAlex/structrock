# Structrock

A cross-platform outcrop point cloud processing system,
capable of acquiring fracture data on suppositional planes cutting through digital outcrop models (DOMs).

Structrock is released under the terms of the BSD license, and thus free for commercial and research use.
Feel free to analyze your own outcrop point cloud data, to add Structrock into your own project.
You are welcomed to make contributions to this project. Email ericrussell@zju.edu.cn, or open an issue here on GitHub for support.

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

Structrock provide a platform on which workflows of processing outcrop point clouds can be designed.
A special algorithm to identify and segment fracture faces is implemented.
Algorithm of restoring a suppositional plane cutting through digital outcrop models (DOMs) is implemented.
Fracture data are acquired from DOMs and displayed.

Structrock can run in both GUI and command-line modes.
In GUI mode, users interact with Structrock to perform certain processing action and provide processing parameters through input dialogs.
In command-line mode, Structrock receive the path to a text file that contains processing action and parameters, separated with ";".
It will do these processing actions one by one until the end or errors occur.

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