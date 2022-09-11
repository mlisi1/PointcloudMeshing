#!/usr/bin/env python

#=====================================================================================================#
#                                        streamer                                                     #
#=====================================================================================================#
# 
# Main node for the RealSense ROS wrapper. It handles topic initialization and message publishing, taking
# the informations from pyrealsense classes. Most of the conversion to ROS messages is handled by already
# existing bridge functions, exception made for pointcloud, which has its own custom functions defined in
# streamer_libs to be cast to PointCloud2 message.
# This node also provides a simple GUI for pointcloud filtering.
#
#------------------------------------------------------------------------------------------------------
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import pyrealsense2 as rs
import numpy as np
import cv2
from lidar.streamer_libs import *
from lidar.GUI import GUI
from cv_bridge import CvBridge
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3dh

#===========================#
#   GUI handling section    #
#===========================#
#
# In this section the GUI is initialized and elements are added.
# To better understand this section, the GUI classes and subclasses are explained in GUI.py
#
#-------------------------------------------------------------------------------------------------------
GUI_handler = GUI('Filters Interface')

#Decimate filter GUI initialization
decimate_check = GUI_handler.add_checkbutton('Decimate Filter:')
decimate_scale = GUI_handler.create_slider('Decimate', 'int', 2,8,1, def_val = 3)

GUI_handler.add_spacer()

#Threshold filter GUI initialization
threshold_check = GUI_handler.add_checkbutton('Threshold Filter:')
threshold_min_scale = GUI_handler.create_slider('Threshold Min', 'int', 0,16,1, def_val = 0)
threshold_max_scale = GUI_handler.create_slider('Threshold Max', 'int', 0,16,1, def_val = 7)

GUI_handler.add_spacer()

#Spatial filter GUI initialization
spatial_check = GUI_handler.add_checkbutton('Spatial Filter:')
spatial_magnitude = GUI_handler.create_slider('Magnitude', 'int', 1,5,1,2)
spatial_alpha = GUI_handler.create_slider('Smooth Alpha', 'float', 0.25, 1, 0.05, 0.5)
spatial_delta = GUI_handler.create_slider('Smooth Delta', 'int', 1, 50, 1, 20)
spatial_hole = GUI_handler.create_slider('Hole Filling', 'int', 0, 5, 1, 0)

GUI_handler.add_spacer(True)

#Play/Pause button
play_button = GUI_handler.add_play_button()

GUI_handler.add_spacer(True)


#Interactions between the enablers and the filter's scales
decimate_check.add_interaction(decimate_scale)
threshold_check.add_interaction(threshold_min_scale)
threshold_check.add_interaction(threshold_max_scale)
spatial_check.add_interaction(spatial_magnitude)
spatial_check.add_interaction(spatial_alpha)
spatial_check.add_interaction(spatial_delta)
spatial_check.add_interaction(spatial_hole)




#Check if we are streaming from camera or not
stream = rospy.get_param('/lidar/stream')
bag_filename = rospy.get_param('/lidar/bag_filename')

bag_filename = "/home/elechim/Documents/D435/car_front_movement_1.bag"


#===============================#
#   check_for_pause(playback)   #
#===============================#
#
# Function used to play/pause the streaming. If the Pause button is pressed, it traps the loop
# while just updating the GUI. 
#
#-------------------------------------------------------------------------------------------------------
def check_for_pause():

    while not play_button.play:

        GUI_handler.tk_routine()

#===============#
#   streamer()  #
#===============#
#
# Main function for the node
#
#------------------------------------------------------------------------------------------------------
def streamer():

    #Initialize pipeline and config
    pipeline = rs.pipeline()
    config = rs.config()


    #If we are not streaming from a camera, play a bag file
    if not stream:

        rs.config.enable_device_from_file(config, bag_filename)

    #Enable the streams and get the needed informations
    config.enable_all_streams()
    profile = pipeline.start(config)    
    device = profile.get_device()    
    playback = rs.playback(device)

    #Get the sensor and the depth scale
    depth_sensor = device.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale() 


    #Initialize classes that will handle poitcloud or frame manipulation later
    colorizer = rs.colorizer()
    align = rs.align(rs.stream.color)

    #Initialize cv2 to ROS bridge
    bridge = CvBridge()

    #Initialize all the needed topics
    rospy.init_node('streamer', anonymous=True)
    color_pub = rospy.Publisher('camera/color', Image, queue_size=1)
    depth_pub = rospy.Publisher('camera/depth', Image, queue_size=1)
    depth_color_pub = rospy.Publisher('camera/depth/color', Image, queue_size=1)
    camera_info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size = 1)
    infrared_pub = rospy.Publisher('camera/infrared', Image, queue_size = 1)
    pc_pub =rospy.Publisher('camera/points', PointCloud2, queue_size = 10)
    
    rate = rospy.Rate(60) # 60hz

    rospy.logwarn("Stream is:" + str(stream))


    #Main loop
    while not rospy.is_shutdown():

        #Check if the pause button has been pressed
        check_for_pause()

        #Get the frame and aligned frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        #Get color, depth and infrared images
        color_image = np.asanyarray(frames.get_color_frame().get_data())
        depth_image = np.asanyarray(frames.get_depth_frame().get_data())
        infrared_image = np.asanyarray(frames.get_infrared_frame().get_data())

        #Get the colorized depth image
        depth_color_frame = colorizer.colorize(frames.get_depth_frame())
        depth_color_image = np.asanyarray(depth_color_frame.get_data())


        #Get the aligned color and depth images
        aligned_depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())
        aligned_color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())
        resized_color_image = cv2.resize(aligned_color_image, dsize=(depth_image.shape[1], depth_image.shape[0]), interpolation=cv2.INTER_AREA)


        #Initialize filters; their values are taken from the GUI handler
        decimate = rs2.decimation_filter(decimate_scale.var.get())
        threshold = rs2.threshold_filter(threshold_min_scale.var.get(), threshold_max_scale.var.get()) 
        spatial = rs2.spatial_filter(spatial_alpha.var.get(), spatial_delta.var.get(), spatial_magnitude.var.get(), spatial_hole.var.get())      

        #Initialize pointcloud
        pc = rs.pointcloud()

        #Frames filtering
        processed_frames = aligned_frames

        if decimate_check.var.get():

            processed_frames = decimate.process(processed_frames).as_frameset()

        if threshold_check.var.get():

            processed_frames = threshold.process(processed_frames).as_frameset()

        if spatial_check.var.get():

            processed_frames = spatial.process(processed_frames).as_frameset()


        #Align the resulting pointcloud to the color frame
        pc.map_to(aligned_frames.get_color_frame())

        #Process the points
        frames_pc = pc.process(processed_frames).as_frameset()

        #Convert pointcloud and texture to PointCloud2 message
        pc_msg = publish_points_cloud(frames_pc, resized_color_image)        

        #Get camera intrinsics
        intrinsic = convert_intrinsics(get_intrinsics(frames))
        camera_info_msg = build_camera_info(get_intrinsics(frames))

        #Image conversion to ROS message
        color_msg = bridge.cv2_to_imgmsg(color_image,  encoding = 'rgb8')
        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding = 'passthrough')
        depth_color_msg = bridge.cv2_to_imgmsg(depth_color_image, encoding = 'passthrough')
        infrared_msg = bridge.cv2_to_imgmsg(infrared_image, encoding = 'passthrough')


        #Publish messages
        color_pub.publish(color_msg)
        depth_pub.publish(depth_msg)
        depth_color_pub.publish(depth_color_msg)
        infrared_pub.publish(infrared_msg)
        camera_info_pub.publish(camera_info_msg)
        pc_pub.publish(pc_msg)

        #Update GUI
        GUI_handler.tk_routine()
        rate.sleep()


if __name__ == '__main__':
    try:
        streamer()
    except rospy.ROSInterruptException:
        pass
