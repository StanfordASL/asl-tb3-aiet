import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors


def best_fit_transform(A: np.ndarray, B: np.ndarray):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Naxm numpy array of corresponding points
      B: Nbxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''
    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, _, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m - 1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.eye(m + 1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src: np.ndarray, dst: np.ndarray, radius=0.01):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''
    ######################################################################
    ######################### YOUR CODE HERE #############################
    # Use the NearestNeighbors class sklearn.neighbors to compute the nearest
    # neighbor of each point in the source dataset to the target dataset
    # Note that by using this class and its methods correctly, you should be
    # able to get the distance between a point and its nearest neighbor,
    # as well as the indices of the nearest neighbor in the target point cloud
    NN = NearestNeighbors(n_neighbors=1, radius=radius)
    NN.fit(dst)
    distances, indices = NN.kneighbors(src, n_neighbors=1, return_distance=True)
    ######################################################################
    return distances.ravel(), indices.ravel()


def icp(
    A: np.ndarray, 
    B: np.ndarray, 
    init_pose=None, 
    max_iterations=20, 
    tolerance=0.001, 
    knn_radius=0.01
) -> np.ndarray:
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''
    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m + 1, A.shape[0]))
    dst = np.ones((m + 1, B.shape[0]))
    src[:m, :] = np.copy(A.T)
    dst[:m, :] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0
    ######################################################################
    ######################### YOUR CODE HERE #############################
    # Write the loop for the ICP algorithm here
    # Follow the steps outlined in the prompt above.
    # Hints:
    # - Use the functions `nearest_neighbor()` and `best_fit_transform()`
    # - src and dst matrices are defined above with shape (m+1,N), while the above
    #   two function expect matrices of shape (N,m)
    for i in range(max_iterations):
        distances, indices = nearest_neighbor(src[:m, :].T, dst[:m, :].T, radius=knn_radius)
        T, _, _ = best_fit_transform(src[:m, :].T, dst[:m, indices].T)
        src = np.dot(T, src)

        error = np.mean(distances)
        print(f"Error at iteration {i}: {error}")

        if abs(prev_error - error) < tolerance:
            break
        prev_error = error
    ######################################################################
    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)

    return T


def open3d_icp(source, target, T_init):
    o3d.geometry.PointCloud.estimate_normals(
        source,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30)
    )

    o3d.geometry.PointCloud.estimate_normals(
        target,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30)
    )

    # Perform point-to-plane ICP
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, 1.0, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
        
    return reg_p2p.transformation