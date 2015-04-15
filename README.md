Bridges between ROS TF messages and Robot Scene Graph Nodes
==========================================================

Overview
--------

Bridges between ROS TF messages and Robot Scene Graph Nodes

Dependencies
------------

 - ROS Hydro
 - BRICS_3D library with HDF5 support. Installation instructions can be found here: http://www.best-of-robotics.org/brics_3d/installation.html
 - Open Scene Graph (optional, recommended)

Compilation
-----------

```
 $ catkin_make -DUSE_OSG=ON

```

Environment Variables
---------------------

Please make sure the following environment variables are set. (The should be ) 

Dependencies to BRICS_3D and HDF5:


| Name          | Description |
| ------------- | ----------- |
| BRICS_3D_DIR  | Points to the installation folder of BRICS_3D. Used within the CMake scripts to discover the BRICS_3D library. |
| HDF5_ROOT     | Points to the installation folder of HDF5. Use it in case it is not into installed to the default folders. |



Usage
-----

To start the example do the following:

```
rosrun sherpa_world_model_tf_bridge sherpa_world_model_tf_bridge
```

The system will be _initialized_. To actually _start_ it go to the web interface at 
localhost:8888 and click on all dark green _start_ buttons.


Licensing
---------

This software is published under a dual-license: GNU Lesser General Public
License LGPL 2.1 and Modified BSD license. The dual-license implies that
users of this code may choose which terms they prefer. Please see the files
called LGPL-2.1 and BSDlicense.


Impressum
---------

Written by Sebastian Blumenthal (blumenthal@locomotec.com)
Last update: 15.04.2015
 



