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
from pc_meshing.GUI import GUI
import numpy as np
from open3d_ros_helper import open3d_ros_helper as o3dh
from pc_meshing.icp import *
from pc_meshing.shared_libs import *
import time
from tqdm import tqdm


#GUI initialization
GUI_handler = GUI("Registration")
filter_out = GUI_handler.add_checkbutton('Filter Outliers:', False)
buffer = GUI_handler.add_spinbox('int', 1, 20, 1, 5)
start = GUI_handler.add_button('Start Registration')
GUI_handler.add_spacer()
fine = GUI_handler.add_checkbutton('Fine Registration', False)
color = GUI_handler.add_checkbutton('Random Colors', False)
voxel = GUI_handler.add_spinbox('float', 0.001, 5.0, 0.005, 0.05)
compute = GUI_handler.add_button('Compute Registration')
GUI_handler.add_spacer()
save = GUI_handler.add_button('Save')
GUI_handler.add_spacer(True)
reset = GUI_handler.add_button('Reset Buffer')
GUI_handler.add_spacer(True)



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
		self.already_filtered = False

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

				pcd = rospc_to_o3dpc(msg)

				pcd = pcd.remove_non_finite_points()

				# o3d.visualization.draw(pcd)

				back = o3dh.o3dpc_to_rospc(pcd, frame_id = 'map')

				self.pc_pub.publish(back)


				self.buffer.append(pcd)

			else:

				rospy.logwarn('::Buffer Full::')
				self.enable = False



	#Empties the buffer
	def reset(self):

		self.buffer = []
		self.enable = False
		self.tr = []
		self.result = None
		self.already_filtered = False


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


			# self.result = self.result.voxel_down_sample(0.5)

			end = time.time()

			rospy.logwarn('::Computation Finished::')
			rospy.logwarn(end-start)


	#Applies multiway registration on the buffer's pointclouds; see ICP library for more detailed informations
	def compute_multiway(self):

		if not self.enable:


			if filter_out.check() and not self.already_filtered:

				rospy.logwarn('::Filtering Outliers::')
				for i in tqdm(range(len(self.buffer))):

					pcd, _ = remove_outlier(self.buffer[i])
					self.buffer[i] = pcd

				self.already_filtered = True


			#compute posegraph
			rospy.logwarn('::Calculating posegraph::')
			pose_graph = multiway_registration(self.buffer, voxel.var.get())


			
			if (fine.var.get() == 1):

				#Optimize posegraph
				rospy.logwarn('::Optimizing posegraph::')
				optimize_posegraph(pose_graph, voxel.var.get())

			#Merge the resulting pointclouds
			self.result = merge(self.buffer, pose_graph, color.check())

			self.result = self.result.voxel_down_sample(0.02)
			rospy.logwarn('::===Completed!===::')

		

	#Publish pointcloud
	def publish(self):
		
		if self.result is not None:

			msg = o3dh.o3dpc_to_rospc(self.result, frame_id = 'map')

			self.pc_pub.publish(msg)



	#Saving function for debugging purposes
	def save(self):

		if self.result is not None:

			o3d.io.write_point_cloud('/home/elechim/Documents/pc.ply', self.result)





def main():

	#Initialize Pointcloud class
	pc = PointCloud()

	#Add commands to the GUI buttons
	start.add_command(pc.enable_callback)
	compute.add_command(pc.compute_multiway)
	save.add_command(pc.save)
	reset.add_command(pc.reset)

	#Initialize node and assign subcriber callback
	rospy.init_node('registration', anonymous = False)
	if (check_for_node('/realsense_streamer')):
		pc_sub = rospy.Subscriber('/camera/points', PointCloud2, pc.pc_callback)

	if (check_for_node('/kinect_streamer')):
		pc_sub = rospy.Subscriber('/kinect/points', PointCloud2, pc.pc_callback)

	#10 fps
	rate = rospy.Rate(30)


	while not rospy.is_shutdown():

		#Publish the merged pointcloud
		pc.publish()

		#Update GUI
		GUI_handler.tk_routine()
		rate.sleep()



if __name__ == '__main__':

	main()