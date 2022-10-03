#=====================================================================================================#
#                                        		registration                                          #
#=====================================================================================================#
# 
# Node handling the pointloud registration process. When active, the streamer node will send
# pointclouds at 1 frame per second, mimicing a sampler. After selecting the buffer size and 
# starting the registration, the subscriber callback stores the frames in the PointCloud() buffer.
# Then, selecting a voxel size and clicking on 'Compute Registration', the ICP algorithms are applied
# and the resulting reconstructed pointcloud is sent to /computed/points.
# If 'fine registration' is checked, the transformations from the ICP pipeline are optimized before
# being applied.
#
#------------------------------------------------------------------------------------------------------
import rospy
from sensor_msgs.msg import Image, PointCloud2
from lidar.GUI import GUI
import numpy as np
from open3d_ros_helper import open3d_ros_helper as o3dh
from lidar.icp import *
from lidar.shared_libs import rospc_to_o3dpc
import time


#GUI initialization
GUI_handler = GUI("Registration")
buffer = GUI_handler.add_spinbox('int', 1, 20, 1, 5)
start = GUI_handler.add_button('Start Registration')
GUI_handler.add_spacer()
fine = GUI_handler.add_checkbutton('Fine Registration', False)
color = GUI_handler.add_checkbutton('Random Colors', False)
voxel = GUI_handler.add_spinbox('float', 0.001, 5.0, 0.005, 0.05)
compute = GUI_handler.add_button('Compute Registration')
GUI_handler.add_spacer()
mesh = GUI_handler.add_button('Mesh')
GUI_handler.add_spacer(True)
reset = GUI_handler.add_button('Reset Buffer')




#===============#
#	PointCloud 	#
#===============#
#
# PointCloud class, handling all the processes from registration and callbacks to
# ICP pipeline and optimization
#
#-------------------------------------------------------------------------------------
class PointCloud:


	def __init__(self):

		self.enable = False
		self.buffer_len = 0
		self.buffer = []	
		self.tr = []
		self.result = None
		self.pc_pub = rospy.Publisher('/computed/points', PointCloud2, queue_size = 10)

	#Enable ROS subscriber callback to store the frames
	def enable_callback(self):

		if len(self.buffer) == 0:

			rospy.logwarn('::Frame registration started::')
			self.buffer_len = buffer.var.get()
			self.enable = True

		else: 

			rospy.logwarn('::Buffer already full::')
		

	#ROS subscriber callback; stores the pointcouds to the buffer
	def pc_callback(self, msg):

		if self.enable:

			if len(self.buffer) < self.buffer_len:

				self.buffer.append(rospc_to_o3dpc(msg))

			else:

				rospy.logwarn('::Buffer Full::')
				self.enable = False



	#Empties the buffer
	def reset(self):

		self.buffer = []
		self.enable = False
		self.tr = []
		self.result = None


	#Applies RANSAC and ICP registration; much slower than multiway registration and thus not used anymore
	def compute(self):		

		if not self.enable:

			start = time.time()

			s = self.buffer[0]			
			new_s = None

			for i in range(1, len(self.buffer)-1):

				t = self.buffer[i]

				transformation = pairwise_RANSAC_ICP((s if i == 1 else new_s), t, voxel.var.get(), (fine.var.get()==1))
				self.tr.append(transformation)

				new_s = t


			self.result = transform(self.buffer, self.tr)

			end = time.time()

			rospy.logwarn('::Computation Finished::')
			rospy.logwarn(end-start)


	#Applies multiway registration on the buffer's pointclouds; see ICP library for more detailed informations
	def compute_multiway(self):

		if not self.enable:

			#compute posegraph
			rospy.logwarn('::Calculating posegraph::')
			pose_graph = multiway_registration(self.buffer, voxel.var.get())

			
			if (fine.var.get() == 1):

				#Optimize posegraph
				rospy.logwarn('::Optimizing posegraph::')
				optimize_posegraph(pose_graph, voxel.var.get())

			#Merge the resulting pointclouds
			self.result = merge(self.buffer, pose_graph, (color.var.get()==1))
			rospy.logwarn('::===Completed!===::')

		

	#Publish pointcloud
	def publish(self):
		
		if self.result is not None:

			msg = o3dh.o3dpc_to_rospc(self.result, frame_id = 'map')

			self.pc_pub.publish(msg)


	# WIP
	# Function used to create the Triangular Mesh from the pointcloud; not yet fully implemented
	def to_3d(self):
		
		if self.result is not None:

			#downsample the pcd
			# value = voxel.var.get()
			self.result = self.result.voxel_down_sample(0.01)

			if not self.result.has_normals():

				self.result.estimate_normals()
				self.result.orient_normals_towards_camera_location()

			# mesh = self.result.compute_convex_hull()


			distances = self.result.compute_nearest_neighbor_distance()
			avg_dist = np.mean(distances)
			value = avg_dist 
			radii = [value, value*2]#, value*4, value*8]

			mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(self.result, o3d.utility.DoubleVector(radii))
			# mesh = mesh.filter_smooth_simple(number_of_iterations=2)

			mesh = mesh.filter_smooth_laplacian(2)


			dec_mesh = mesh.simplify_quadric_decimation(100000)

			dec_mesh = dec_mesh.remove_degenerate_triangles()
			dec_mesh = dec_mesh.remove_duplicated_triangles()
			dec_mesh = dec_mesh.remove_duplicated_vertices()
			dec_mesh = dec_mesh.remove_non_manifold_edges()


			o3d.visualization.draw(dec_mesh)

	#Saving function for debugging purposes
	def save(self):

		if self.result is not None:

			o3d.io.write_point_cloud('/home/elechim/Documents/car.ply', self.result)





def main():

	#Initialize Pointcloud class
	pc = PointCloud()

	#Add commands to the GUI buttons
	start.add_command(pc.enable_callback)
	compute.add_command(pc.compute_multiway)
	mesh.add_command(pc.to_3d)
	reset.add_command(pc.reset)

	#Initialize node and assign subcriber callback
	rospy.init_node('registration', anonymous = False)
	pc_sub = rospy.Subscriber('/camera/points', PointCloud2, pc.pc_callback)

	#10 fps
	rate = rospy.Rate(10)


	while not rospy.is_shutdown():

		#Publish the merged pointcloud
		pc.publish()

		#Update GUI
		GUI_handler.tk_routine()
		rate.sleep()



if __name__ == '__main__':

	main()