#=====================================================================================================#
#                                        kinect_streamer                                              #
#=====================================================================================================#
# 
# Main node for the Kinect v2 ROS wrapper. It handles topic initialization and message publishing, taking
# the informations from freenect2 classes. Pointcloud is computed starting from a points vector and the
# color frame.
#
#------------------------------------------------------------------------------------------------------
import rospy
import freenect2 as fk2
import numpy as np

from open3d_ros_helper import open3d_ros_helper as o3dh
from pc_meshing.shared_libs import rospc_to_o3dpc
import pc_meshing.kinect_libs as kl
import open3d as o3d

from sensor_msgs.msg import Image, PointCloud2

from cv_bridge import CvBridge
import cv2



def main():

	#initialize node and topics
	rospy.init_node('kinect_streamer', anonymous = False)
	color_pub = rospy.Publisher('/kinect/color', Image, queue_size = 10)
	depth_pub = rospy.Publisher('/kinect/depth', Image, queue_size = 10)
	ir_pub = rospy.Publisher('/kinect/ir', Image, queue_size = 10)
	pc_pub = rospy.Publisher('kinect/points', PointCloud2, queue_size = 10)
	rate = rospy.Rate(60)
	pc = o3d.geometry.PointCloud()
	bridge = CvBridge()

	#check for Kinect
	try:
		device = fk2.Device()
	except fk2.NoDeviceError:
		print(fk2.NoDeviceError)
		rospy.logerr('::No Kinect connected::')
		exit()


	#initialize listener
	listener =kl.QueueFrameListener()
	device.start(listener)
	frames = {}

	while not rospy.is_shutdown():			

		#get frames
		frames = kl.wait_for_frames_no_ir(device)
	
		#separate frames
		rgb, depth = frames[fk2.FrameType.Color], frames[fk2.FrameType.Depth]

		#compute big depth
		undistorted, registered, big_depth = device.registration.apply(rgb, depth, with_big_depth=True, enable_filter = True)

		color_image = np.asanyarray(rgb.to_image())
		depth_image = np.asanyarray(depth.to_image())
		# ir_image = np.asanyarray(ir.to_image())

		#compute points
		# points = device.registration.get_points_xyz_array(undistorted)
		big_points = device.registration.get_big_points_xyz_array(big_depth)

		# big_color_data = rgb.to_array()
		# depth_data = undistorted.to_array()
		# big_depth_data = big_depth.to_array()
		# color_data = registered.to_array()

		
		#compute Pointcloud
		pc = kl.color_pointcloud(big_points, color_image, pc)

		#cast to ROS msg
		color_msg = bridge.cv2_to_imgmsg(color_image,  encoding = 'rgb8')
		depth_msg = bridge.cv2_to_imgmsg(depth_image,  encoding = 'passthrough')
		# ir_msg = bridge.cv2_to_imgmsg(ir_image,  encoding = 'passthrough')
		pc_msg = o3dh.o3dpc_to_rospc(pc, frame_id = 'map')

		#publish message
		color_pub.publish(color_msg)
		depth_pub.publish(depth_msg)
		# ir_pub.publish(ir_msg)
		pc_pub.publish(pc_msg)

		rate.sleep()	

	
	device.close()


if __name__ == '__main__':

	try:

		
		main()

	except rospy.ROSInterruptException:
		pass

	

