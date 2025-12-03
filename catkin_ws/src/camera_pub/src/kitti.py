#! /usr/bin/env python3
from data_utils import *
from publish_utils import *
from kitti_util import *
from misc import *
import os
from collections import deque

# use data path from the ros container
DATA_PATH = '/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync'
# ego car dimensions from official KITTI doc
EGOCAR = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73],
                    [2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])

class Object():
    def __init__(self, center):
        # use deque to store fixed-length location history
        self.locations = deque(maxlen=20)
        self.locations.appendleft(center)
    
    def update(self, center, displacement, yaw_change):
        for i in range(len(self.locations)):
            x0, y0 = self.locations[i]
            x1 = x0 * np.cos(yaw_change) + y0 * np.sin(yaw_change) - displacement
            y1 = -x0 * np.sin(yaw_change) + y0 * np.cos(yaw_change)
            self.locations[i] = np.array([x1, y1])
        
        if center is not None:
            # update
            self.locations.appendleft(center)

    def reset(self):
        self.locations = deque(maxlen=20)

if __name__ == "__main__": 
    frame = 0
    rospy.init_node("kitti_node", anonymous=True)
    '''
    3 steps to publish a topic:
    1. create publisher
    2. load data / create message
    3. publish message
    '''
    # 1. Create Publishers
    cam_pub = rospy.Publisher("kitti_cam", Image, queue_size=10)
    pcl_pub = rospy.Publisher("kitti_point_cloud", PointCloud2, queue_size=10)
    ego_pub = rospy.Publisher("ego_car_marker", MarkerArray, queue_size=10)
    imu_pub = rospy.Publisher("kitti_imu", Imu, queue_size=10)
    gpu_pub = rospy.Publisher("kitti_gps", NavSatFix, queue_size=10)
    box3d_pub = rospy.Publisher("kitti_3d_boxes", MarkerArray, queue_size=10)
    loc_pub = rospy.Publisher('kitti_loc', MarkerArray, queue_size=10)
    dist_pub = rospy.Publisher('kitti_dist', MarkerArray, queue_size=10)

    # initialize cv bridge
    bridge = CvBridge()

    rate = rospy.Rate(10)  # 10hz

    df_tracking = read_trackings('/data/kitti/tracking/training/label_02/0000.txt')
    calib = Calibration('/data/kitti/RawData/2011_09_26/', from_video=True)

    # # initialize ego car object 
    # ego_car = Object()
    # prev_imu_data = None

    # create tracker dict to store all the objects we want to track from the past to present 
    # while centers only stores objects appeared in the current frame
    # {tracker_id: Object} 
    tracker = {}
    prev_imu_data = None


    while not rospy.is_shutdown():
        # 2. Load data
        df_tracking = df_tracking[df_tracking.type.isin(['Car', 'Truck', 'Van', 'Tram', 'Pedestrian', 'Cyclist'])]
        boxes_2d = np.array(df_tracking[df_tracking.frame==frame][['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
        types = np.array(df_tracking[df_tracking.frame==frame]['type'])
        track_ids = np.array(df_tracking[df_tracking.frame==frame]['track_id'])

        img = read_camera(os.path.join(DATA_PATH, "image_02/data/%010d.png" %frame))
        # load point cloud into a matrix of n x 4
        point_cloud = read_point_cloud(os.path.join(DATA_PATH, "velodyne_points/data/%010d.bin" %frame))
        imu_data = read_imu(os.path.join(DATA_PATH, "oxts/data/%010d.txt" %frame))

        # load 3d boxes
        boxes_3d = np.array(df_tracking[df_tracking.frame==frame][['height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']])
        
        corners_3d_velos = []
        centers = {}    # store track id: center pairs
        centers[-1] = np.array([0.0, 0.0])  # ego car center
        minPQDs = []
        for track_id, box_3d in zip(track_ids, boxes_3d):
            corners_3d_box = compute_3d_box_cam2(*box_3d)
            corners_3d_velo = calib.project_rect_to_velo(corners_3d_box.T)
            minPQDs.append(min_distance_cuboids(EGOCAR, corners_3d_velo))
            corners_3d_velos.append(corners_3d_velo)
            # mean of 8 * 3 matrix -> mean of x, y, z -> only take x, y
            centers[track_id] = np.mean(corners_3d_velo, axis = 0)[:2]

        # if this is the first frame, initialize:
        if prev_imu_data is None:
            for track_id in centers:
                tracker[track_id] = Object(centers[track_id])
        else:
            displacement = 0.1 * np.linalg.norm(imu_data[['vf', 'vl']])
            yaw_change = float((imu_data.yaw - prev_imu_data.yaw).iloc[0])
            # update all the detected objects in the cur frame
            for track_id in centers:
                if track_id in tracker: # if it was previously detectetd
                    tracker[track_id].update(centers[track_id], displacement, yaw_change)
                else:   # if this was its first appearance
                    tracker[track_id] = Object(centers[track_id])
            # update those obj which was previously detected but for some reason is being occulted in the cur fram
            for track_id in tracker:
                if track_id not in centers:
                    tracker[track_id].update(None, displacement, yaw_change)

        prev_imu_data = imu_data

            
        # 3. Publish
        publish_camera(cam_pub, bridge, img, boxes_2d, types)
        publish_point_cloud(pcl_pub, point_cloud)
        publish_ego_car(ego_pub)
        publish_imu_data(imu_pub, imu_data)
        publish_gps(gpu_pub, imu_data)
        publish_3d_boxes(box3d_pub, corners_3d_velos, types)
        publish_loc(loc_pub, tracker, centers)
        publish_dist(dist_pub, minPQDs)
        
        rospy.loginfo("publishing frame %d" %frame)
        rate.sleep()
        frame += 1
        # loop back to beginning
        if frame == 154:
            frame = 0
            for track_id in tracker:
                tracker[track_id].reset()
