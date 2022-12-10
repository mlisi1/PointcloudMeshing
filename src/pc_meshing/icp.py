#=====================================================================================================#
#                                               icp                                                   #
#=====================================================================================================#
# 
# Library used by the registration node. Here are defined the functions that apply various 
# registration algorithms.
#
#------------------------------------------------------------------------------------------------------
import open3d as o3d
import numpy as np
import copy
import random
import rospy
from tqdm import tqdm



def remove_outlier(pcd, nb_neighbors = 100, std_ratio = 0.2, paint_outlier = True):

    _, ind = pcd.remove_statistical_outlier(nb_neighbors, std_ratio)

    inlier = pcd.select_by_index(ind)
    outlier = pcd.select_by_index(ind, invert = True)

    if paint_outlier:
        outlier.paint_uniform_color((1, 0, 0))

    return inlier, outlier




def pairwise_RANSAC_ICP(source, target, voxel_size, refined = True):

    #Downsample inputs
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size) 

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)

    #Estimate normals with KDTree algorithm
    source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5

    #Compute FPFH features with KDTree algorithm
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(source_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(target_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    distance_threshold = voxel_size * 1.5


    print(":: RANSAC registration on downsampled point clouds.")
    print(":: Since the downsampling voxel size is %.3f," % voxel_size)
    print(":: we use a liberal distance threshold %.3f." % distance_threshold)

    #Aplly RANSAC registration with feature matchin and estimate a transformation
    ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                        source_down,
                        target_down, 
                        source_fpfh, 
                        target_fpfh, 
                        False, 
                        distance_threshold,
                        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 
                        3, 
                        [ 
                            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
                            # o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.1)
                        ], 
                        o3d.pipelines.registration.RANSACConvergenceCriteria(4000000,1.0))


    #If the refined registration is not required, return the transformation
    if refined is False:        

        return ransac_result.transformation

    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print(":: clouds to refine the alignment. This time we use a strict")
    print(":: distance threshold %.3f." % distance_threshold)


    #Apply ICP algorithm initialized with the RANSAC estimanted transformation to refine it
    icp_result = o3d.pipelines.registration.registration_icp(
                        source, 
                        target, 
                        distance_threshold, 
                        ransac_result.transformation,
                        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 20))


    return icp_result.transformation



def transform(pcs, trasfs):

    if len(pcs) > 1 and len(trasfs) > 0:

        merged = o3d.geometry.PointCloud()

        #Apply transformations in the correct order
        for j in range(len(trasfs)):

            tr = trasfs[j]

            for i in range(len(pcs)-(len(trasfs)-j)):

                pc = pcs[i]

                pc.transform(tr)

        #Merge the different pointclouds 
        for pc in pcs:
            
            merged += pc


        return merged



def pairwise_registration(source, target, voxel_size):

    #ICP pipeline:
    #   -Coarse ICP Point to Plane
    #   -Fine ICP Point to Plane
    #   -Coarse ICP Point to Point
    #   -Coarse Colored ICP
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, voxel_size * 5, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, voxel_size * 0.1,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, voxel_size * 3,
        icp_fine.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    icp_colored = o3d.pipelines.registration.registration_colored_icp(
                        source, target, voxel_size * 2, 
                        icp_fine.transformation,
                        o3d.pipelines.registration.TransformationEstimationForColoredICP())

    transformation_icp = icp_colored.transformation

    #Get Matrix informations
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, voxel_size * 0.1,
        icp_fine.transformation)


    return transformation_icp, information_icp





def multiway_registration(buff, voxel_size):

    #Initialize pose graph
    buffer = buff
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.eye(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))

    #For every pointcloud in buffer
    for i in tqdm(range(len(buffer)-1)):

        #Compute normals (needed for Point to Plane)
        if not buffer[i].has_normals():

            buffer[i].estimate_normals()
            buffer[i].orient_normals_towards_camera_location()

        if not buffer[i+1].has_normals():

            buffer[i+1].estimate_normals()
            buffer[i+1].orient_normals_towards_camera_location()

        #Downsample
        source = buffer[i].voxel_down_sample(voxel_size)
        target = buffer[i+1].voxel_down_sample(voxel_size)

        #Apply pairwise registration
        icp_tr, icp_info = pairwise_registration(source, target, voxel_size)

    
        #Update posegraph with nodes and edge
        if not (i+1 >= len(buffer)):  # odometry case

            odometry = np.dot(icp_tr, odometry)
            pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
            pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(i, i+1, icp_tr, icp_info, uncertain=False))

        else:  # loop closure case

            pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(i, i+1, icp_tr, icp_info,  uncertain=True))

    return pose_graph

#Optimize posegraph
def optimize_posegraph(pose_graph, voxel_size):


    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=voxel_size * 0.5,
        edge_prune_threshold=0.25,
        reference_node=0,
        preference_loop_closure = 0.1)

    
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationGaussNewton(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)






def merge(buff, pose_graph, random_color = True):

    

    buffer = buff

    if random_color:

        colored = []
        i=0

        for pc in buffer:

            pc_copy = copy.deepcopy(pc)
            pc_copy.paint_uniform_color([random.random(), random.random(), random.random()])
            colored.append(pc_copy)
            colored[i].transform(pose_graph.nodes[i].pose)
            i+=1

    #Apply transformations for every pointcloud
    for i in range(len(buffer)):
        buffer[i].transform(pose_graph.nodes[i].pose)

    result = o3d.geometry.PointCloud()

    #Merge pointclouds
    for i in range(len(buffer)):

        result += colored[i] if random_color else buffer[i]

    return result







#For debug purposes only
if __name__ == '__main__':

    voxel_size = 0.05

    # p1 = o3d.io.read_point_cloud("/home/elechim/Documents/tests/r1.ply")
    # p2 = o3d.io.read_point_cloud("/home/elechim/Documents/tests/r2.ply")

    p1 = o3d.io.read_point_cloud("/home/elechim/Documents/D435/m1.ply")
    p2 = o3d.io.read_point_cloud("/home/elechim/Documents/D435/m2.ply")

    p1_d = p1.voxel_down_sample(voxel_size)
    p2_d = p2.voxel_down_sample(voxel_size)

    # print(p1+1)


    pose_graph = multiway_registration([p1_d, p2_d], voxel_size)


    optimize_posegraph(pose_graph, voxel_size)



    merge([p1, p2], pose_graph, False)



    