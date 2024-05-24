import numpy as np
from scipy import spatial

DEFAULT_RANGE_SENSING = 2.0
DEFAULT_NB_NEIGHBORS = 3
DEFAULT_R_AGENT = 0.069

def compute_neihgborhood(drone_id, drone_pos, others_pos, metric='voronoi', **kwargs):
    match metric:
        case "eucledian":
            sensing_range = kwargs.get('range', DEFAULT_RANGE_SENSING)
            distances = np.linalg.norm(others_pos - drone_pos, axis=1)
            indices = np.nonzero(distances < sensing_range)[0]
            neighbors = [(i if i<drone_id else i+1, others_pos[i]) for i in indices]
        case "topological":
            nb = kwargs.get('count', DEFAULT_NB_NEIGHBORS)
            distances = np.linalg.norm(others_pos - drone_pos, axis=1)
            indices = np.argsort(distances)[:nb]
            neighbors = [(i if i<drone_id else i+1, others_pos[i]) for i in indices]
        case "voronoi":
            points = np.concatenate((others_pos, drone_pos.reshape(1,3)))
            indptr_neig, neighbors = spatial.Delaunay(points, qhull_options="QJ").vertex_neighbor_vertices
            neighbors = [(j if j<drone_id else j+1, others_pos[j]) for j in neighbors[indptr_neig[-2]:indptr_neig[-1]]]
        case "vlos":
            sensing_range = kwargs.get('range', np.inf)
            r_agent = kwargs.get('r_agent', DEFAULT_R_AGENT)
            # Keep neighbors that are within the sensing range
            distances = np.linalg.norm(others_pos - drone_pos, axis=1)
            indices = np.where(distances < sensing_range)[0]
            distances = distances[indices]
            # Get headings and distances to all neighbors
            headings = (others_pos[indices] - drone_pos) / distances[:, np.newaxis]
            indices = np.argsort(distances)
            neighbors = []
            while len(indices) > 0:
                n_index = indices[0] 
                if indices[0] >= drone_id:
                    n_index += 1 
                neighbors.append((n_index, others_pos[indices[0]]))
                # Check if the neighbor is within the field of view
                d_ij = distances[indices[0]]
                u_ij = headings[indices[0]]
                r_ij = r_agent/d_ij
                to_remove = [0]
                for i in range(1, len(indices)):
                    # Looping through all others
                    d_ik = distances[indices[i]]
                    u_ik = headings[indices[i]]
                    r_ik = r_agent/d_ik
                    # First condition (d_ij < d_ik) is already satisfied by the sorting
                    if np.linalg.norm(u_ij-u_ik) < (r_ij + r_ik):
                        to_remove.append(i)
                indices = np.delete(indices, to_remove)
            
        case _:
            neighbors = []

    return neighbors

def get_viewing_dir(drone_pos, neighbors_pos, metric='average', **params):
    """
    Compute the viewing direction of the drone based on the neighbors
    with the selected algorithm.

    Args:
        drone (Drone): The selected drone
        neighbors (list[DroneNeighbor]): The neighbors of the drone
        algo (str): The algorithm to use to compute the viewing direction
    """
    assert metric in ["average", "outter", "tangent_plane", "convex_hull", "yaw_diff"], "Algorithm {0} not supported".format(algo)
    # Test if any neighbors
    if len(neighbors_pos) == 0:
        return None
    return eval(metric)(drone_pos, neighbors_pos, params)

def average(drone_pos, neighbors_pos, params):
    """
    Compute the desired viewing direction based on the centroid of the neighbors.

    1. Find the centroid (mean) of the neighbors
    2. Set the viewing direction to the opposite of the centroid
    3. Don't forget to normalize the vector

    Args:
        drone (Drone): The selected drone
        neighbors (list[DroneNeighbor]): The neighbors of the drone
        params (dict): {'in_2d': True or False}
    """
    centroid = np.mean(neighbors_pos, axis=0)
    vec_to_centroid = centroid - drone_pos
    viewing_dir = -vec_to_centroid / np.linalg.norm(vec_to_centroid)
    # If 2D, set z component to zero
    if params.get('in_2d', False):
        viewing_dir[2] = 0
    return viewing_dir

def tangent_plane(drone_pos, neighbors_pos, params):
    """
    Compute the desired viewing direction based on the tangent plane of the neighbors.

    1. Find the centroid (mean) of the neighbors
    2. Compute covariance matrix of the neighbors
    3. Apply PCA and keep the last eigenvector (with smallest eigenvalue)
    4. This vector gives the normal to the tangent plane
    5. Set the viewing direction to the opposite of the normal

    Args:
        drone (Drone): The selected drone
        neighbors (list[DroneNeighbor]): The neighbors of the drone
        params (dict): {'in_2d': True or False}
    """
    in_2d = params.get('in_2d', False)
    if in_2d:
        neighbors_pos = neighbors_pos[:, :2]
    centroid = np.mean(neighbors_pos, axis=0)
    neighbors_centered = neighbors_pos - centroid
    cov = np.sum([np.outer(n, n) for n in neighbors_centered], axis=0)
    eig_val, eig_vectors = np.linalg.eig(cov)
    eig_val, eig_vectors = np.real(eig_val), np.real(eig_vectors)
    sorted_indices = np.argsort(eig_val)
    normal = eig_vectors[sorted_indices[0]]
    # Printing stuff
    # print("Covariance matrix:")
    # for i in range(3):
    #     print(cov[i])
    # print("Eigenvectors:")
    # for i in range(3):
    #     print(eig_vectors[i])
    # Verify if eigenvalues are close to each other
    if abs(eig_val[sorted_indices[1]] - eig_val[sorted_indices[0]]) < 0.01:
        print("WARNING :: Eigenvalues are too close to each other, averaging the two smallest")
        normal = np.mean(eig_vectors[sorted_indices[:2]], axis=0)
    if in_2d:
        viewing_dir = np.hstack((-np.sign(np.dot(normal, centroid-drone_pos[:2]))*normal, 0))
    else:
        viewing_dir = -np.sign(np.dot(normal, centroid-drone_pos))*normal
    return viewing_dir

def convex_hull(drone_pos, neighbors_pos, params):
    """
    Compute the desired viewing direction based on the convex hull of the neighbors.

    1. Find the convex hull of the neighbors + the drone
    2. Set the viewing direction either to the normal of the adjacent faces
       or to the one of the visible faces
    3. Don't forget to normalize the vector

    Args:
        drone (Drone): The selected drone
        neighbors (list[DroneNeighbor]): The neighbors of the drone
        params (dict): {'faces': 'adjacent' or 'visible', 'in_2d': True or False}
    """
    # Check if # neighbors is enough
    in_2d = params.get('in_2d', False)
    if len(neighbors_pos) < 2:
        print("WARNING :: Not enough neighbors to compute convex hull")
        return None
    elif len(neighbors_pos) == 2:
        # Do outter2 metric (max angle)
        dists = neighbors_pos - drone_pos
        if in_2d:
            dists = dists[:, :2]
        # Compute dot product between all combination of neighbors
        smaller = np.inf
        smaller_indices = (0, 0)
        for i in range(len(neighbors_pos)):
            for j in range(i+1, len(neighbors_pos)):
                d = np.dot(dists[i], dists[j])
                if d < smaller:
                    smaller = d
                    smaller_indices = (i, j)
        viewing_dir = -(dists[smaller_indices[0]] + dists[smaller_indices[1]]) / 2
    else:
        # Convex hull
        points = neighbors_pos
        idx_drone = len(neighbors_pos)
        faces_type = params.get('faces', 'adjacent')
        # Prepare standard options for qhull solver
        qhull_options = '' if in_2d else 'tJ'
        ndim = 2 if in_2d else 3
        if in_2d:
            points = points[:, :2]
        if faces_type == 'adjacent':
            points = np.concatenate((points, [drone_pos[:ndim]]))
            hull = spatial.ConvexHull(points, qhull_options=f'Q{qhull_options}')
            # Check if drone is in the convex hull
            if idx_drone not in hull.vertices:
                return None
            # Compute the normal of the adjacent faces
            adj_idx = np.where(hull.simplices == idx_drone)[0]
            normals = hull.equations[adj_idx, :ndim]
            # Compute the viewing direction
            viewing_dir = np.mean(normals, axis=0)
        elif faces_type == 'visible':
            # Edge case with 3 neighbors
            if len(neighbors_pos) == 3 and not in_2d:
                centroid = np.mean(points[:3], axis=0)
                normal = np.cross(points[1] - points[0], points[2] - points[0])
                viewing_dir = -np.sign(np.dot(normal, centroid - points[-1]))*normal
            else:
                hull = spatial.ConvexHull(points, qhull_options=f'Q{qhull_options}')
                # Compute the centroid of each face
                hull_centroids = np.mean(points[hull.simplices], axis=1)
                # Find closest to the drone
                closest_idx = np.argmin(np.linalg.norm(hull_centroids - drone_pos[:ndim], axis=1))
                closest_normal = hull.equations[closest_idx, :ndim]
                visible_normals = []
                for i in range(len(hull.equations)):
                    if np.dot(hull.equations[i, :ndim], closest_normal) > 0:
                        visible_normals.append(hull.equations[i, :ndim])
                # Compute the viewing direction
                viewing_dir = np.mean(visible_normals, axis=0)
        else:
            raise ValueError("Invalid value for 'faces' in params")
    
    # Add z component to zero if in 2D
    if in_2d:
        viewing_dir = np.hstack((viewing_dir, 0))
    return viewing_dir / np.linalg.norm(viewing_dir)

# def yaw_diff(drone, neighbors, params):
#     #Go through all the neighbours of the drone and compute the average yaw
#     num_neighbours = len(neighbors)
#     average_yaw_diff = 0
#     if num_neighbours > 0:
#             # Loop through the neighbours
#             for i in range(num_neighbours):
#                 yaw_diff = neighbour_yaws[i] - drone_yaw
#                 if yaw_diff > np.pi:
#                     yaw_diff -= 2*np.pi
#                 elif yaw_diff < -np.pi:
#                     yaw_diff += 2*np.pi
#                 average_yaw_diff += yaw_diff

#             # Divide by number of neighbours
#             average_yaw_diff /= num_neighbours
    
#         # # Add the yaw_diff
#         # average_yaw_diff += yaw_diff_pos*0.2

