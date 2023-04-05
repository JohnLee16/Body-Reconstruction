import numpy as np
import open3d as o3d
import sys



def passthroughfilter(pointcloud, axis, min = sys.float_info.min, max = sys.float_info.max):
    '''
    passthrouth filter for point cloud

    Parameters
    ----------
    pointcloud: point cloud in open3d
    axis: x('X'), y('Y') or z('Z')
    min: min value of axis
    max: max value of axis

    Return
    ------
    o3d.geometry.PointCloud()
    '''
    point_stack = np.asarray(pointcloud.points)
    color_stack = np.asarray(pointcloud.colors)

    if axis == 'x' or axis == 'X':
        ax = point_stack[:, 0]
    elif axis == 'y' or axis == 'Y':
        ax = point_stack[:, 1]
    elif axis == 'z' or axis == 'Z':
        ax = point_stack[:, 2]
    else:
        raise Exception("Sorry, no numbers below zero")
    
    field1 = ax < max
    field2 = ax > min
    
    field = np.logical_and(field1, field2)
    point_stack = point_stack[field, :]
    color_stack = color_stack[field, :]
    
    output_pointcloud = o3d.geometry.PointCloud()
    output_pointcloud.points = o3d.utility.Vector3dVector(point_stack)
    output_pointcloud.colors = o3d.utility.Vector3dVector(color_stack)
    return output_pointcloud
