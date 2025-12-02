#! /usr/bin/env python3

import cv2
import numpy as np
import pandas as pd

IMU_COLUMN_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw',
                    'vn', 've', 'vf', 'vl', 'vu',
                    'ax', 'ay', 'az', 'af', 'al', 'au',
                    'wx', 'wy', 'wz', 'wf', 'wl', 'wu',
                    'posacc', 'velacc', 'navstat', 'numsats',
                    'posmode', 'velmode', 'orimode']

TRACKING_COLUMN_NAMES = ['frame', 'track_id', 'type', 'truncated', 'occluded',
                'alpha', 'bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom',
                'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']

def read_camera(path):
    """Reads an image from the given path."""
    return cv2.imread(path)

def read_point_cloud(path):
    """Reads a point cloud from a binary file and returns it as an n x 4 numpy array."""
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)

def read_imu(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = IMU_COLUMN_NAMES
    return df

def read_trackings(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = TRACKING_COLUMN_NAMES
    df = df[df.type.isin(['Car', 'Truck', 'Van', 'Tram', 'Pedestrian', 'Cyclist'])]
    return df