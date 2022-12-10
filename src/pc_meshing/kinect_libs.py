#=====================================================================================================#
#                                     kinect_libs                                                     #
#=====================================================================================================#
# Utility library used by the kinect_streamer.py node.
#------------------------------------------------------------------------------------------------------
from queue import Queue
import freenect2 as fk2
import numpy as np
import open3d as o3d


#Custom QueueFrameListener with increased max size; the default one throws
# exception during normal runtime due to full queue
class QueueFrameListener(object):
    def __init__(self, maxsize=12):
        self.queue = Queue(maxsize=maxsize)
    def __call__(self, frame_type, frame):
        # you can also check is the queue is full using .full method
        if self.queue.qsize() >= 6: # change the number of frames you want to use 
            # basically remove an item to accomodate the newer frames
            _ = self.get() # or, self.queue.get(True, timeout)
        self.queue.put_nowait((frame_type, frame))
        # print('remove')
    def get(self, timeout=False):
        return self.queue.get(True, timeout)



def get_frames(device):

	try:
		frame_type, frame = device.color_frame_listener.get(None)

	except Empty:
		raise NoFrameReceivedError()

	return frame_type, frame



#=======================#
#	wait_for_frames()	#
#=======================#
# 
# Function calling the freenect2 frame listener returning a frame array
#
#-----------------------------------------------------------------------
def wait_for_frames(device):

	frames = {}

	while not (fk2.FrameType.Color in frames and fk2.FrameType.Depth in frames and fk2.FrameType.Ir in frames):

		try:
			type_, frame = device.color_frame_listener.get(None)

		except Empty:
			raise NoFrameReceivedError()
		
		frames[type_] = frame


	return frames


#===========================#
#	wait_for_frames_no_ir()	#
#===========================#
# 
# Function calling the freenect2 frame listener returning a frame array but
# without ir frame (becaus it may cause slowdowns)
#
#--------------------------------------------------------------------------
def wait_for_frames_no_ir(device):

	frames = {}

	while not (fk2.FrameType.Color in frames and fk2.FrameType.Depth in frames):

		type_, frame = get_frames(device)
		frames[type_] = frame

	return frames



#=======================#
#	color_pointcloud()	#
#=======================#
#
# Function that takes the Kinect's color image, the (big) points computed
# with freenect2 library and casts them to an open3d PointCloud class
#
#------------------------------------------------------------------------
def color_pointcloud(points, image, pc):

	#discard last 2 lines (for some reason points is 1920x1082)
	points = points[:-2,:]
	color = image.reshape((points.shape[1]*points.shape[0], 3))
	#normalise image
	color = color/255

	#cast to float64 (otherwise Vector3dVector becomes a bottleneck)
	color = color.astype('float64')
	points = points.reshape((points.shape[0]*points.shape[1], 3)).astype('float64')

	pc.points = o3d.utility.Vector3dVector(points)
	pc.colors = o3d.utility.Vector3dVector(color)	

	
	# pc = pc.uniform_down_sample(3)


	return pc


