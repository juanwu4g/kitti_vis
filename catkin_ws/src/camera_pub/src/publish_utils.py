 #! /usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
import tf
import numpy as np
import cv2

FRAME_ID = "map"
BOX_COLOR = {'Car': (255, 255, 0), 'Truck': (255, 255, 0), 'Van': (255, 255, 0), 'Tram': (255, 255, 0), 'Pedestrian': (0, 256, 255), 'Cyclist': (141, 40, 255)}
LIFETIME = 0.1  # seconds

def publish_camera(cam_pub, bridge, img, boxes, types):
    for typ, box in zip(types, boxes):
        top_left = int(box[0]), int(box[1])
        bottom_right = int(box[2]), int(box[3])
        cv2.rectangle(img, top_left, bottom_right, BOX_COLOR[typ], 2)
    cam_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

def publish_point_cloud(pcl_pub, point_cloud):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = FRAME_ID
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))
     
def publish_ego_car(ego_car_pub):
    """
    Publishes left and right 45 degree FOV markers for the ego car and ego car model mesh
    """
    # use marker array to publish multiple markers
    marker_array = MarkerArray()

    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time.now()

    marker.id = 0
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration()
    marker.type = Marker.LINE_STRIP

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.2

    marker.points = []
    # Coordinates for right 45 degree FOV line
    marker.points.append(Point(10, -10, 0))
    marker.points.append(Point(0, 0, 0))
    # Coordinates for left 45 degree FOV line
    marker.points.append(Point(10, 10, 0))

    marker_array.markers.append(marker)

    """
    Publishes a 3D model of the ego car
    """
    mesh_marker = Marker()
    mesh_marker.header.frame_id = FRAME_ID
    mesh_marker.header.stamp = rospy.Time.now()

    mesh_marker.id = 1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = "package://camera_pub/models/bmwx5.dae"
    
    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = -1.73  # Adjust height to align with ground

    q = tf.transformations.quaternion_from_euler(np.pi / 2, 0, np.pi)
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]

    mesh_marker.scale.x = 1.0
    mesh_marker.scale.y = 1.0
    mesh_marker.scale.z = 1.0

    mesh_marker.color.r = 0.9
    mesh_marker.color.g = 0.9
    mesh_marker.color.b = 0.9
    mesh_marker.color.a = 0.9

    marker_array.markers.append(mesh_marker)    
    ego_car_pub.publish(marker_array)

def publish_imu_data(imu_pub, imu_data):
    imu = Imu()
    imu.header.frame_id = FRAME_ID
    imu.header.stamp = rospy.Time.now()

    # Convert roll, pitch, yaw to quaternion
    q = tf.transformations.quaternion_from_euler(
        float(imu_data.roll.iloc[0]),
        float(imu_data.pitch.iloc[0]),
        float(imu_data.yaw.iloc[0])
    )
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]

    # Set linear accelerations (front, left, up)
    imu.linear_acceleration.x = float(imu_data.af.iloc[0])
    imu.linear_acceleration.y = float(imu_data.al.iloc[0])
    imu.linear_acceleration.z = float(imu_data.au.iloc[0])

    # set angular velocities (front, left, up)
    imu.angular_velocity.x = float(imu_data.wf.iloc[0])
    imu.angular_velocity.y = float(imu_data.wl.iloc[0])
    imu.angular_velocity.z = float(imu_data.wu.iloc[0])

    imu_pub.publish(imu)

def publish_gps(gps_pub, imu_data):
    gps = NavSatFix()
    gps.header.frame_id = FRAME_ID
    gps.header.stamp = rospy.Time.now()

    gps.latitude = float(imu_data.lat.iloc[0])
    gps.longitude = float(imu_data.lon.iloc[0])
    gps.altitude = float(imu_data.alt.iloc[0])

    gps_pub.publish(gps)

def publish_3d_boxes(box3d_pub, corners_3d_velos, types):  
    marker_array = MarkerArray()
    for i, corners_3d_velo in enumerate(corners_3d_velos):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.id = i
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(LIFETIME)
        marker.type = Marker.LINE_LIST

        b, g, r = BOX_COLOR[types[i]]
        marker.color.r = r / 255.0
        marker.color.g = g / 255.0
        marker.color.b = b / 255.0
        marker.color.a = 1.0
        marker.scale.x = 0.1

        marker.points = []
        # Define the 12 edges of the 3D bounding box
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # bottom face
            (4, 5), (5, 6), (6, 7), (7, 4),  # top face
            (0, 4), (1, 5), (2, 6), (3, 7)   # vertical edges
        ]
        for start, end in edges:
            p_start = Point(*corners_3d_velo[start])
            p_end = Point(*corners_3d_velo[end])
            marker.points.append(p_start)
            marker.points.append(p_end)
        marker_array.markers.append(marker)

        # Add text marker for object type
        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = rospy.Time.now()

        text_marker.id = i + 1000  # Ensure unique ID
        text_marker.action = Marker.ADD
        text_marker.lifetime = rospy.Duration(LIFETIME)
        text_marker.type = Marker.TEXT_VIEW_FACING

        p = np.mean(corners_3d_velo, axis=0) # upper front left corner

        text_marker.pose.position.x = p[0]
        text_marker.pose.position.y = p[1]
        text_marker.pose.position.z = p[2] + 1.5

        text_marker.text = types[i]

        text_marker.scale.x = 1
        text_marker.scale.y = 1
        text_marker.scale.z = 1

        b, g, r = BOX_COLOR[types[i]]
        text_marker.color.r = r / 255.0
        text_marker.color.g = g / 255.0
        text_marker.color.b = b / 255.0
        text_marker.color.a = 1.0
        marker_array.markers.append(text_marker)

    box3d_pub.publish(marker_array)

def publish_loc(loc_pub, tracker, centers):
    marker_array = MarkerArray()

    for track_id in centers:
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(LIFETIME)
        marker.type = Marker.LINE_STRIP
        marker.id = track_id

        marker.scale.x = 0.2

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []
        for loc in tracker[track_id].locations:
            marker.points.append(Point(loc[0], loc[1], 0))

        marker_array.markers.append(marker)

    loc_pub.publish(marker_array)

def publish_dist(dist_pub, minPQDs):   
    marker_array = MarkerArray()

    for i, (minP, minQ, min_distance) in enumerate(minPQDs):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.id = i
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(LIFETIME)
        marker.type = Marker.LINE_STRIP

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        marker.scale.x = 0.1

        marker.points = []
        marker.points.append(Point(minP[0], minP[1], 0))
        marker.points.append(Point(minQ[0], minQ[1], 0))

        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = rospy.Time.now()

        text_marker.id = i + 1000  # Ensure unique ID
        text_marker.action = Marker.ADD
        text_marker.lifetime = rospy.Duration(LIFETIME)
        text_marker.type = Marker.TEXT_VIEW_FACING

        P = (minP + minQ) / 2  # midpoint
        text_marker.pose.position.x = P[0]
        text_marker.pose.position.y = P[1]
        text_marker.pose.position.z = 0.0

        text_marker.text = f"{min_distance:.2f} m"
        text_marker.scale.x = 1
        text_marker.scale.y = 1
        text_marker.scale.z = 1

        text_marker.color.r = 1.0
        text_marker.color.g = 0.5
        text_marker.color.b = 0.0
        text_marker.color.a = 0.8
        marker_array.markers.append(text_marker)

    dist_pub.publish(marker_array)