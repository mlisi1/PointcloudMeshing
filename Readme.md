# Car 3D Scanner

This package has the objective to obtain a 3D reconstruction of a car, starting from the outputs of one or more depth cameras, by applying filters and ICP algorithms.

## Nodes:

* ## streamer.py:
The only node present at the moment is the streamer.py node. It acts as a ROS wrapper for RealSense cameras by using the pyrealsense2 library. The following topics are published during the runtime of the node:
- /camera/color
- /camera/depth
- /camera/depth/color
- /camera/infrared
- /camera/points
- /camera/camera_info

All the topics are published directly, exception made for the /camera/points topic, which, being the pointcloud, is preprocessed and then published.
To launch the node, a launchfile has been made, ```roslaunch lidar lidar```, which opens this node and rviz with its relative configuration to correctly visualize the pointcloud.
Two launch parameters have been implemented at the moment:
- **stream**: if *True*, the streaming starts from the camera, if connected
- **bag_filename**: when *stream* is False, the streaming is enabled from a bag file; this is the filepath

Lastly, a simple GUI is provided when launching the node: it has the purpose of letting the user enable/disable filters and modify their values. It also is able to pause or resume the stream.
