#=====================================================================================================#
#                                               shared_libs                                           #
#=====================================================================================================#
# 
# Generic utility libraries
#
#------------------------------------------------------------------------------------------------------
import rosnode
import ros_numpy
import numpy as np
import open3d as o3d



#Function taken from open3d helper; modified due to color handling compatibility issues
def rospc_to_o3dpc(rospc, remove_nans=False):
    """ covert ros point cloud to open3d point cloud
    Args: 
        rospc (sensor.msg.PointCloud2): ros point cloud message
        remove_nans (bool): if true, ignore the NaN points
    Returns: 
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
    """
    field_names = [field.name for field in rospc.fields]
    is_rgb = 'rgb' in field_names
    is_rgb_separated = 'r' in field_names

    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(rospc).ravel()

    if is_rgb:

        cloud_array = ros_numpy.point_cloud2.split_rgb_field(cloud_array)
        is_rgb_separated = True

     
    
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]
    if is_rgb:
        cloud_npy = np.zeros(cloud_array.shape + (4,), dtype=np.float)
    else: 
        cloud_npy = np.zeros(cloud_array.shape + (3,), dtype=np.float)
    
    cloud_npy[...,0] = cloud_array['x']
    cloud_npy[...,1] = cloud_array['y']
    cloud_npy[...,2] = cloud_array['z']
    o3dpc = o3d.geometry.PointCloud()

    if len(np.shape(cloud_npy)) == 3:
        cloud_npy = np.reshape(cloud_npy[:, :, :3], [-1, 3], 'F')
    o3dpc.points = o3d.utility.Vector3dVector(cloud_npy[:, :3])

    #Use 3 separate channels instead of one 'rgb' channel
    if is_rgb_separated:

        
        r = cloud_array['r']
        g = cloud_array['g']
        b = cloud_array['b']

        

        rgb_npy = np.asarray([r, g, b])
        rgb_npy = rgb_npy.astype(np.float)

        if rgb_npy.max() > 1.0:

            rgb_npy = rgb_npy/255


        rgb_npy = np.swapaxes(rgb_npy, 0, 1)
        o3dpc.colors = o3d.utility.Vector3dVector(rgb_npy)



    return o3dpc


#===================#
#   check_for_node  #
#===================#
#
# Checks if the specified node is active or not
#
#-----------------------------------------------
def check_for_node(node):

    nodes = rosnode.get_node_names()

    active = False

    for active_node in nodes:

        if active_node == node:

            active = True

    return active
