#!/usr/bin/env python

#=====================================================================================================#
#                                     realsense_streamer                                              #
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
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, Imu

import pyrealsense2 as rs

import numpy as np
import cv2
from cv_bridge import CvBridge

from pc_meshing.realsense_libs import *
from pc_meshing.shared_libs import check_for_node
from pc_meshing.GUI import GUI

import pstats
import cProfile

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
decimate_check = GUI_handler.add_checkbutton('Decimate Filter:', True)
decimate_scale = GUI_handler.create_slider('Decimate', 'int', 2,8,1, def_val = 6)

GUI_handler.add_spacer()

#Threshold filter GUI initialization
threshold_check = GUI_handler.add_checkbutton('Threshold Filter:', True)
threshold_min_scale = GUI_handler.create_slider('Threshold Min', 'int', 0,16,1, def_val = 0)
threshold_max_scale = GUI_handler.create_slider('Threshold Max', 'int', 0,16,1, def_val = 4)

GUI_handler.add_spacer()

#Spatial filter GUI initialization
spatial_check = GUI_handler.add_checkbutton('Spatial Filter:', False)
spatial_magnitude = GUI_handler.create_slider('Magnitude', 'int', 1,5,1,2)
spatial_alpha = GUI_handler.create_slider('Smooth Alpha', 'float', 0.25, 1, 0.05, 0.5)
spatial_delta = GUI_handler.create_slider('Smooth Delta', 'int', 1, 50, 1, 20)
spatial_hole = GUI_handler.create_slider('Hole Filling', 'int', 0, 5, 1, 0)

GUI_handler.add_spacer()

#Temporal filter GUI initialization
temporal_check = GUI_handler.add_checkbutton('Temporal Filter:', False)
temporal_alpha = GUI_handler.create_slider('Smooth Alpha', 'float', 0, 1, 0.1, 0.4)
temporal_delta = GUI_handler.create_slider('Smooth Delta', 'int', 1, 100, 1, 20)
temporal_persistency = GUI_handler.create_slider('Persistency', 'int', 0, 8, 1, 3)

GUI_handler.add_spacer()

#Hole filling filter GUI initialization
hole_filling_check = GUI_handler.add_checkbutton('Hole Filling Filter:', False)
hole_filling_mode = GUI_handler.create_slider('Mode', 'int', 0, 2, 1)

GUI_handler.add_spacer(True)

d2d_check = GUI_handler.add_checkbutton('Depth to Disparity', True)


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
temporal_check.add_interaction(temporal_alpha)
temporal_check.add_interaction(temporal_delta)
temporal_check.add_interaction(temporal_persistency)
hole_filling_check.add_interaction(hole_filling_mode)


#Check if we are streaming from camera or not
stream = rospy.get_param('/realsense_streamer/stream')
bag_filename = rospy.get_param('/realsense_streamer/bag_filename')

bag_filename = "/home/elechim/Documents/tests/slide.bag"



#Initialize filters; their values are taken from the GUI handler
#This will be used with filters_pipeline() function
filters_dict = {(rs2.decimation_filter, decimate_check):[decimate_scale.var], 
                        (rs2.threshold_filter, threshold_check):[threshold_min_scale.var, threshold_max_scale.var],
                        (rs2.disparity_transform, d2d_check):[True],
                        (rs2.spatial_filter, spatial_check):[spatial_alpha.var, spatial_delta.var, spatial_magnitude.var, spatial_hole.var],
                        (rs2.temporal_filter, temporal_check):[temporal_alpha.var, temporal_delta.var, temporal_persistency.var],
                        (rs2.disparity_transform, d2d_check):[False],
                        (rs2.hole_filling_filter, hole_filling_check):[hole_filling_mode.var]
                        }



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
    rospy.init_node('realsense_streamer', anonymous=False)
    color_pub = rospy.Publisher('camera/color', Image, queue_size=1)
    depth_pub = rospy.Publisher('camera/depth', Image, queue_size=1)
    depth_color_pub = rospy.Publisher('camera/depth/color', Image, queue_size=1)
    camera_info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size = 1)
    infrared_pub = rospy.Publisher('camera/infrared', Image, queue_size = 1)
    pc_pub =rospy.Publisher('camera/points', PointCloud2, queue_size = 3)
    imu_pub = rospy.Publisher('camera/IMU', Imu, queue_size = 1)
    

    rospy.logwarn("Stream is:" + str(stream))


    #Main loop
    while not rospy.is_shutdown():

        #If registration node is up, start publishing at 1 frame per second, else stream at 60
        rate = rospy.Rate(1 if check_for_node('/registration') else 60)       

        #Check if the pause button has been pressed
        check_for_pause()

        #Get the frame and aligned frames
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        #Get color, depth and infrared images
        color_image = np.asanyarray(frames.get_color_frame().get_data())
        depth_image = np.asanyarray(frames.get_depth_frame().get_data())
        infrared_image = np.asanyarray(frames.get_infrared_frame().get_data())
        time = frames.get_timestamp()
    

        #Get the colorized depth image
        depth_color_frame = colorizer.colorize(frames.get_depth_frame())
        depth_color_image = np.asanyarray(depth_color_frame.get_data())


        #Get the aligned color and depth images
        aligned_depth_image = np.asanyarray(aligned_frames.get_depth_frame().get_data())
        aligned_color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())
        resized_color_image = cv2.resize(aligned_color_image, dsize=(depth_image.shape[1], depth_image.shape[0]), interpolation=cv2.INTER_AREA)       


        #Initialize pointcloud
        pc = rs.pointcloud()

        processed_frames = aligned_frames

        #Filters pipeline            
        processed_frames = filters_pipeline(processed_frames, filters_dict)


        #Align the resulting pointcloud to the color frame
        pc.map_to(aligned_frames.get_color_frame())

        #Process the points
        frames_pc = pc.process(processed_frames).as_frameset()

        #Convert pointcloud and texture to PointCloud2 message        
        pc_msg = publish_points_cloud(frames_pc, resized_color_image, time)        

       
        #Get camera intrinsics
        intrinsic = convert_intrinsics(get_intrinsics(frames))
        camera_info_msg = build_camera_info(get_intrinsics(frames))

        #Image conversion to ROS message
        imu_msg = get_IMU_msg(frames)
        color_msg = bridge.cv2_to_imgmsg(color_image,  encoding = 'rgb8')
        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding = 'passthrough')
        depth_color_msg = bridge.cv2_to_imgmsg(depth_color_image, encoding = 'passthrough')
        infrared_msg = bridge.cv2_to_imgmsg(infrared_image, encoding = 'passthrough')


        #Publish messages
        imu_pub.publish(imu_msg)
        color_pub.publish(color_msg)
        depth_pub.publish(depth_msg)
        depth_color_pub.publish(depth_color_msg)
        infrared_pub.publish(infrared_msg)
        camera_info_pub.publish(camera_info_msg)
        pc_pub.publish(pc_msg)

        #Code used to have informations on the nodes benchmarks; for optimization purposes
        # stats = pstats.Stats(pr)        
        # stats.sort_stats(pstats.SortKey.TIME)
        # stats.dump_stats(filename = '/home/elechim/Documents/profiling.prof')


        #Update GUI
        GUI_handler.tk_routine()
        rate.sleep()




if __name__ == '__main__':
    try:

        with cProfile.Profile() as pr:
            streamer()        


    except rospy.ROSInterruptException:
        pass
