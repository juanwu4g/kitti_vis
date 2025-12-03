#! /usr/bin/env python3
import numpy as np

def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    """
    Return: 3 x n in cam2 coordinate
    """
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
    z_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    corners_3d_cam2 += np.vstack([x, y, z])
    return corners_3d_cam2

def distance_point_to_segment(P, A, B):
    """ 
    Calculate the distance from point P to line segment AB
    P, A, B are numpy arrays of shape (2,)
    Returns the point Q in AB that is closest to P and the distance from P to Q
    """
    AP = P - A
    BP = P - B
    AB = B - A
    if np.dot(AP, AB) >= 0 and np.dot(-AB, BP) >= 0:
        # perpendicular foot falls on the segment
        return np.abs(np.cross(AB, AP)) / np.linalg.norm(AB), np.dot(AB, AP) / np.dot(AB, AB) * AB + A
    # perpendicular foot falls outside the segment
    d_PA = np.linalg.norm(AP)
    d_PB = np.linalg.norm(BP)
    if d_PA < d_PB:
        return d_PA, A
    return d_PB, B

def min_distance_cuboids(cub1, cub2):
    '''
    Calculate the minimum distance between two non-overlapping cuboids of shape (8, 3)
    they are projected to 2D plane (x, y) and the minimum distance of their edges is calculated
    '''
    min_distance = float('inf')
    # point in cub1 to cub2 segments
    for i in range(4):
        for j in range(4):
            d, Q = distance_point_to_segment(cub1[i, :2], cub2[j, :2], cub2[j+1, :2])
            if d < min_distance:
                min_distance = d
                minP = cub1[i, :2]
                minQ = Q

    # repeat for cub2 points to cub1 segments
    for i in range(4):
        for j in range(4):
            d, Q = distance_point_to_segment(cub2[i, :2], cub1[j, :2], cub1[j+1, :2])
            if d < min_distance:
                min_distance = d
                minP = cub2[i, :2]
                minQ = Q
    return minP, minQ, min_distance