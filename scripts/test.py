import open3d as o3d
import numpy as np



def remove_outlier(pcd, nb_neighbors = 100, std_ratio = 0.2):

	_, ind = pcd.remove_statistical_outlier(nb_neighbors, std_ratio)

	inlier = pcd.select_by_index(ind)
	outlier = pcd.select_by_index(ind, invert = True)

	outlier.paint_uniform_color((0, 0, 1))

	return inlier, outlier




pcd = o3d.io.read_point_cloud('/home/elechim/Documents/box_full_515.ply')


pcd = pcd.remove_non_finite_points()


# bb = pcd.get_axis_aligned_bounding_box()
# bb2 = pcd3.get_axis_aligned_bounding_box()

# print(bb2.max_bound)

# bb.max_bound = [0.3, .5, 1.3]
# bb.min_bound = [-1, -0.64794268,  0.1]

# bb.max_bound = [0.3, .5, 1.3]
# bb.min_bound = [-1, -0.25,  0.1]

# bb2.max_bound = [0.3, .5, 1.1]
# bb2.min_bound = [-0.6, -0.64794268,  0.34348412]



# pcd = pcd.crop(bb)




pcd, outlier = remove_outlier(pcd, 30, 0.7)

voxel = 0.05


distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
value = avg_dist *2

print(value, value/2, value*2)

radii = [value/2, value, value*2, value*4]

mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

# mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=13, linear_fit = False)


        
dec_mesh = mesh.remove_degenerate_triangles()
dec_mesh = dec_mesh.remove_duplicated_triangles()
dec_mesh = dec_mesh.remove_duplicated_vertices()
dec_mesh = dec_mesh.remove_non_manifold_edges()



# wire = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)


o3d.visualization.draw(dec_mesh)