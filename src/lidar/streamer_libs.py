#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=====================================================================================================#
#                                   streamer_libs                                                     #
#=====================================================================================================#
# Utility library mostly containing custom conversion functions or pointcloud processing
#------------------------------------------------------------------------------------------------------
import open3d as o3d
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField, Imu
from geometry_msgs.msg import Vector3
import numpy as np
import rospy
import random
import pyrealsense2 as rs2
from scipy.integrate import cumtrapz
from open3d_ros_helper import open3d_ros_helper as o3dh
import cv2

#===================#
#   get_IMU_msg     #
#===================#
#
# Gets IMU data from the realsense frame and casts it to ROS IMU message.
#
#-------------------------------------------------------------------------
def get_IMU_msg(frames):

    #Initialize messages
    accel = Vector3()
    gyro = Vector3()
    msg = Imu()

    #For every frame
    for frame in frames:

        #Find motion frame
        if frame.is_motion_frame():

            #Get IMU data
            motion = frame.as_motion_frame()
            profile = motion.get_profile()
            data = motion.get_motion_data()

            #Update Gyro data
            if profile.stream_name() == 'Gyro':
                
                gyro.x = data.x
                gyro.y = data.y
                gyro.z = data.z
                
            #Update Accel data
            if profile.stream_name() == 'Accel':

                accel.x = data.x
                accel.y = data.y
                accel.z = data.z

    #Assign fields
    msg.angular_velocity = gyro
    msg.linear_acceleration = accel
    msg.header.frame_id = 'map'

    return msg






#===========================================#
#   uv_to_rgb(texture_coordinate, texture)  #
#===========================================#
#
# Function used to map the uv coordinates (texture coordinate argument) to the actual rgb value
# of the texture. At the moment, the function supports only uv map arrays created from the 
# get_texture_coordinates() method from a rs2::points object
#
#-----------------------------------------------------------------------------------------------

def uv_to_rgb(text_coords, texture):

    #store texture size
    size = (texture.shape[1]-1, texture.shape[0]-1)

    #Convert values from structured type to float32
    uv_map = text_coords.view(('float32', len(text_coords.dtype.names)))

    #Clip values to [0-1] to avoid texture repetition
    uv_map = np.clip(uv_map, 0, 1)

    #Get texture index mapping by multyplying uv values to image size
    xy = np.multiply(uv_map, size)
    xy = xy.astype('int')

    x = xy[:,0]
    y = xy[:,1]

    #Fetch the RGB values from the texture
    rgb = np.asanyarray(texture[y,x])


    return rgb


#===========================#
#   publish_single_point()  #
#===========================#
#
# Debug function used to understand how to send in a PointCloud2 message XYZ coordinates along with
# RGB values to encode the texture with points. It only sends a PointCloud2 message with a single
# point randmly changing color.
#
#--------------------------------------------------------------------------------------------------
def publish_single_point():

    #Initialize the array with the point coordinates and color
    arr = np.array((0.0, 0.0, 5.0, random.random(), random.random(), random.random()), dtype = '6float32')

    #Message creation
    msg = PointCloud2()
    msg.header.frame_id = "map"
    step = arr[0].nbytes 
    
    msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', step, PointField.FLOAT32, 1),
            PointField('z', step*2, PointField.FLOAT32, 1),
            PointField('r', step*3, PointField.FLOAT32, 1),
            PointField('g', step*4, PointField.FLOAT32, 1),
            PointField('b', step*5, PointField.FLOAT32, 1),
            ]


    msg.height = 1
    msg.width = 1 
    msg.is_bigendian = False
    msg.point_step = step * 6
    msg.row_step = step * len(arr)
    msg.is_dense = False
    msg.data = arr.tobytes()   

    return msg


#===================================#
#   subscribe_pointcloud(message)   #
#===================================#
#
# Function created for subscriber nodes; used to convert the PointCloud2 message to an array of
# points and colors. It is used as an alternative function to the provided conversion functions
# from the open3d helper conversion function. 
# Currently a work in progress.
#
#----------------------------------------------------------------------------------------------
def subscribe_pointcloud(msg):

    #Get the bytes and the size of the original array
    data = msg.data
    length = len(data)
    point_step = msg.point_step

    #like array; used by numpy to organize data
    like = np.empty(shape = (int(length/point_step), 6), dtype = '6float32')

    #Read the raw bytes and organize them in an array
    array = np.frombuffer(data, '6float32', like = like)

    #Retrieve the point and color
    x = np.array(array[:,0], dtype = 'float32')
    y = np.array(array[:,1], dtype = 'float32')
    z = np.array(array[:,2], dtype = 'float32')
    r = np.array(array[:,3], dtype = 'float32')
    g = np.array(array[:,4], dtype = 'float32')
    b = np.array(array[:,5], dtype = 'float32')

    points = np.array((x,y,z)).T
    color = np.array((r,g,b)).T


    return points


# def publish_points(points):

#     x = np.array(points[:,0],dtype='float32')
#     y = np.array(points[:,1],dtype='float32')
#     z = np.array(points[:,2],dtype='float32')


#     msg = PointCloud2()
#     msg.header.frame_id = 'map'
#     msg.height = 1
#     msg.width = len(points)
#     step = points[0][0].nbytes  

#     msg.fields = [
#         PointField('x', 0, PointField.FLOAT32, 1),
#         PointField('y', step, PointField.FLOAT32, 1),
#         PointField('z', step*2, PointField.FLOAT32, 1),   
#         ]        

#     msg.is_bigendian = False
#     msg.point_step = step * 3
#     msg.row_step = step * len(points)
#     msg.is_dense = False
#     msg.data = points.tostring()

#     return msg


#===========================================#
#   publish_pointcloud(frame, color_image)  #
#===========================================#
#
# Custom conversion function created to go from RealSense specific classes to PointCloud2. The two
# arguments are, respectively, a RealSense pointcloud, treated as frameset, and the color image
# the former pointcloud has been mapped to, resized to the depth image size.
#.#-------------------------------------------------------------------------------------------------
def publish_points_cloud(frame, color_image, time):


        # no_color = [255, 0, 0]

        #Compute vertices and texture coordinates (uv maps) from the pointcloud
        pc = frame[0].as_points() 
        text_coords = np.array(pc.get_texture_coordinates())
        vertices = np.array(pc.get_vertices())

        #Convert values from structured type to float32
        vertices = vertices.view(('float32', len(vertices.dtype.names)))

        x = np.array(vertices[:,0],dtype='float32')
        y = np.array(vertices[:,1],dtype='float32')
        z = np.array(vertices[:,2],dtype='float32')


        #get pixel array from uv coordinates and texture
        rgb = uv_to_rgb(text_coords, color_image)
        rgb = np.asanyarray(rgb, dtype = 'float32')

        #Normalize from 0-255 to 0.0-1.0 (otherwise color issues happen in rviz)
        rgb =  (rgb - np.min(rgb)) / (np.max(rgb) - np.min(rgb))

        r = np.array(rgb[:,0])
        g = np.array(rgb[:,1])
        b = np.array(rgb[:,2])

        #Take all the values together
        data = np.array((x,y,z,r,g,b), dtype = 'float32').T    

        #Message creation
        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.header.seq = int(time)
        msg.height = 1
        msg.width = len(data)
        step = data[0][0].nbytes  

        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', step, PointField.FLOAT32, 1),
            PointField('z', step*2, PointField.FLOAT32, 1),
            PointField('r', step*3, PointField.FLOAT32, 1),
            PointField('g', step*4, PointField.FLOAT32, 1),
            PointField('b', step*5, PointField.FLOAT32, 1),
            ]        

        msg.is_bigendian = False
        msg.point_step = step * 6
        msg.row_step = step * len(data)
        msg.is_dense = False
        msg.data = data.tostring()

        # rospy.logwarn(vertices.shape + color_image.shape)


        return msg



#===========================#
#   get_intrinsics(frame)   #
#===========================#
#
# A simple function that retrieves the intrinsics informations from the frame sent.
#
#------------------------------------------------------------------------------------------------
def get_intrinsics(frame):

    #Get the depth frame and profile
    depth_frame = frame.get_depth_frame()
    depth_profile = depth_frame.get_profile()

    #Get the intrinsics
    intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

    return intrinsics


#===================================#
#   convert_intrinsics(intrinsic)   #
#===================================#
#
# Function created to convert RealSense intrinsic class to open3d intrinsics
#
#--------------------------------------------------------------------------------------------------
def convert_intrinsics(intrinsic):

    o3d_in = o3d.camera.PinholeCameraIntrinsic(

            intrinsic.width,
            intrinsic.height,
            intrinsic.fx,
            intrinsic.fy,
            intrinsic.ppx,
            intrinsic.ppy

        )


    return o3d_in


#===================================#
#   build_camera_info(intrinsic)    #
#===================================#
#
# Conversion function to go from RealSense intrinsic class to ROS message CameraInfo()
#
#---------------------------------------------------------------------------------------------------
def build_camera_info(intrinsic): 
    
        camera_info = CameraInfo()

        camera_info.width = intrinsic.width
        camera_info.height = intrinsic.height
        camera_info.distortion_model = str(intrinsic.model)
        cx = intrinsic.ppx
        cy = intrinsic.ppy
        fx = intrinsic.fx
        fy = intrinsic.fy
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
        return camera_info 


#===============================================#
#   get_pc(depth_image, color_image, intrinsic) #
#===============================================#
#
# Old function; used to compute an open3d pointcloud starting from depth and color images.
# It has been chosen not to be used (yet) for a considerable discrepancy from the actual pointcloud
# to the one created using the images.
#
#-----------------------------------------------------------------------------------------
def get_pc(depth_image, color_image, intrinsic): 

    color = o3d.geometry.Image(color_image)
    depth = o3d.geometry.Image(depth_image)   

    pcd = o3d.geometry.PointCloud()

    rgbd = o3d.geometry.RGBDImage().create_from_color_and_depth(color, depth, convert_rgb_to_intensity = False) 
    pcd = o3d.geometry.PointCloud().create_from_rgbd_image(rgbd, intrinsic)

    return pcd