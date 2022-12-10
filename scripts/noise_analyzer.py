import rospy
import numpy as np
import open3d as o3d

import open3d_ros_helper as o3dh

from sensor_msgs.msg import Image, PointCloud2
import numpy as np
from open3d_ros_helper import open3d_ros_helper as o3dh
from pc_meshing.shared_libs import *
import time

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


#initialize publisher
pc_pub = rospy.Publisher('/noise/square', PointCloud2, queue_size = 10)


#3D plot function
def plot(square):

	fig = plt.figure()
	ax = plt.axes(projection='3d')


	x, y, z = square[:,0], square[:,1], square[:,2]

	ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5)	

	#set limits to max and min values
	ax.set_xlim(np.min(x), np.max(x))
	ax.set_ylim(np.min(y), np.max(y))
	ax.set_zlim(np.min(z), np.max(z))


	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	ax.legend()
	plt.show()


z = []


def pc_callback(msg):

	
	if len(z)<1:

		#get points 
		pc = rospc_to_o3dpc(msg)
		pc = pc.remove_non_finite_points()
		points = np.asanyarray(pc.points)


		#check which node is active and apply cropping with the right values
		if (check_for_node('/realsense_streamer')):

			squared = points[np.where(points[:,0]<0.1)]
			squared = squared[np.where(squared[:,0]>-0.1)]

			squared = squared[np.where(squared[:,1]<0.0)]
			squared = squared[np.where(squared[:,1]>-0.1)]

		if (check_for_node('/kinect_streamer')):

			squared = points[np.where(points[:,0]<0.1)]
			squared = squared[np.where(squared[:,0]>-0.1)]

			squared = squared[np.where(squared[:,1]<0.2)]
			squared = squared[np.where(squared[:,1]>0.1)]

		

		z.append(squared)


		#send back square
		pc2 = o3d.geometry.PointCloud()
		pc2.points = o3d.utility.Vector3dVector(squared)
		msg2 = o3dh.o3dpc_to_rospc(pc2, frame_id = 'map')
		pc_pub.publish(msg2)



if __name__ == '__main__':


	rospy.init_node('noise_analyzer', anonymous = False)
	if (check_for_node('/realsense_streamer')):
		pc_sub = rospy.Subscriber('/camera/points', PointCloud2, pc_callback)

	if (check_for_node('/kinect_streamer')):
		pc_sub = rospy.Subscriber('/kinect/points', PointCloud2, pc_callback)



	#10 fps
	rate = rospy.Rate(30)

	while not rospy.is_shutdown():

		if len(z)>0:

			plot(z[0])

		rate.sleep()