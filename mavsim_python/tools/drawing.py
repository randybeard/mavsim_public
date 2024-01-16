import numpy as np

def rotate_points(points, R):
    "Rotate points by the rotation matrix R"
    rotated_points = R @ points
    return rotated_points

def translate_points(points, translation):
    "Translate points by the vector translation"
    translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
    return translated_points

def points_to_mesh(points, index):
    """"
    Converts points to triangular mesh
    Each mesh face is defined by three 3D points
        (a rectangle requires two triangular mesh faces)
    """
    points = points.T
    mesh = np.array([[points[index[0,0]],points[index[0,1]],points[index[0,2]]]])
    for i in range(1, index.shape[0]):
        tmp = np.array([[points[index[i,0]], points[index[i,1]], points[index[i,2]]]])
        mesh = np.concatenate((mesh, tmp), axis=0)
    return mesh
