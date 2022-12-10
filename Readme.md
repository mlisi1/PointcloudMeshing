# PointCloud Meshing

This package has the objective to obtain a 3D reconstruction of a car, starting from the outputs of one or more depth cameras, by applying filters and ICP algorithms.

## Nodes:

* ## realsense_streamer.py:
This node acts as a ROS wrapper for RealSense cameras by using the pyrealsense2 library. The following topics are published during the runtime of the node:
- /camera/color
- /camera/depth
- /camera/depth/color
- /camera/infrared
- /camera/points
- /camera/camera_info
- /camera/IMU

All the topics are published directly, exception made for the /camera/points topic, which, being the pointcloud, is preprocessed and then published.
To launch the node, a launchfile has been made, ```roslaunch pc_meshing realsense.launch```, which opens this node and rviz with its relative configuration to correctly visualize the pointcloud.
Two launch parameters have been implemented at the moment:
- **stream**: if *True*, the streaming starts from the camera, if connected
- **bag_filename**: when *stream* is False, the streaming is enabled from a bag file; this is the filepath

Lastly, a simple GUI is provided when launching the node: it has the purpose of letting the user enable/disable filters and modify their values. It also is able to pause or resume the stream.
The filters implemented at the moment are:
- Decimation Filter
- Threshold Filter
- Spatial Filter
- Temporal Filter
- Hole Filling Filter
- Depth To Disparity 


* ## kinect_streamer.py:
It acts as a ROS wrapper for the Kinect v2, relying on ```freenect2``` and ```libfreenect2``` libraries. 
The following topics are published during runtime:

- /kinect/color
- /kinect/depth
- /kinect/ir
- /kinect/points

* ## noise_analyzer.py:
A simple node created to help visualize PointClouds noise. It subscribes on the streamer /points/ topic (either realsense or kinect) and collects points in a user defined section.
The section is then sent back to /square/points, to check everything is fine, and a 3D plot is generated.

* ## registration.py:
This node has the purpose to take the pointclouds published by either realsense_streamer.py or kinect_streamer.py, save them in a buffer, perform reconstruction ad publish the final result.
When starting the node with ```rosrun pc_meshing registration.py```, the streamer node will slow its publishing rate to 1 frame per second, thus skipping frames. This is to mimic sampling on the stream. If 'Start Registration' is clicked, the frames will be saved in a buffer defined by the user via the first spinbox. 
During buffer filling, every element in the buffer is temporarly sent on /computed/points. This helps seeing what actually is in the buffer.
When the buffer is full, the user can click on 'Compute Registration'; this will apply the following ICP pipeline on the pointclouds:
- Coarse ICP Point to Plane
- Fine ICP Point to Plane
- Coarse ICP Point to Point
- Coarse Colored ICP
The second spinbox is used to choose a voxel size: this is how much the pointclouds in the buffer will be downsized and will affect precision and speed. 
After the pipeline, an open3d ```PoseGraph``` is created; if the 'Fine Registration' checkbutton is active, ```open3d.pipelines.registration.global_optimization()``` is performed to refine it.
Then the transformations in the ```PoseGraph``` are applied to the pointclouds in the buffer, which are then merged and published in ```/computed/points```. If 'Random Color' is active, every pointcloud in the buffer will have different random colors. This is useful when trying to see the different pointclouds.




## Changelog:

* ## Update 2 - 10/12/2022 - Version 1.2.0:
* ### realsense_streamer.py:
- Changed node name from streamer.py to realsense_streamer.py
- Added Depth To Disparity filter (only works with D series cameras)
- Filters application is now handled by the ```filters_pipeline()``` function

* ### kinect_streamer.py:
- Added node
- Added color, depth and ir streaming
- Added pointcloud streaming on /kinect/points

* ### registration.py:
- Added outlier filtering option before PoseGraph computation
- Added non finite points filtering (Kinect gives error without filtering)
- Now during buffer filling, fragments are sent to /computed/points
- Save function added
- When started now the node automatically subscribes to either /kinect/points or /camera/points

* ### noise_analyzer.py:
- Added node
- Node automatically check which topic to subscribe (either realsense or kinect)
- The isolated section is sent to /square/points

* ### realsense_libs:
- Changed libraries name from streamer_libs to realsense_libs 
- Added filters pipeline function 

* ### kinect_libs:
- Added library
- Added custom Queue for frame listener
- Added function that handles frames
- Added function that casts from points array and image to open3d PointCloud

* ### icp:
- Added outliers filtering function




* ## Update 1 - 03/10/2022:
* ### streamer.py:
- Added Temporal Filter
- Added Hole Filling Filter
- Added IMU data publishing
- Streamer fps now change wether or not the **registration.py** node is up or not

* ### registration.py:
- Added node
- Custom ```PointCoud``` class created
- Fast multiway registration via open3d posegraphs

* ### streamer_libs:
- pyrealsese2 frame to ROS Imu message bridge function added

* ### GUI:
- Added ```check()``` method to ```Checkbutton``` class
- Added ```Button``` class
- Added ```Spinbox``` class

* ### shared_libs:
- Added library
- Added function that check wether or not a node is active
- Added custom ROS PointCloud2 message to open3d pointcloud bridge function


 
