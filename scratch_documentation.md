1. docker compose up

2. docker start/attache to continue working

3. xhost +local:docker to allow display

4. on second terminal: docker exec -it my_ros_noetic bash

5. kitti2bag -t 2011_09_26 -r 0005 raw_synced . to convert to ros readable

6. upgrade numpy

7. rqt_bag kitti_2011_09_26_drive_0005_synced.bag to visualize

8. pip install opencv-python

9. root@jw4g-M16:/catkin_ws/src# catkin_create_pkg camera_pub rospy to create ros package

10. root@jw4g-M16:/catkin_ws# catkin_make

11. root@jw4g-M16:/catkin_ws# source devel/setup.bash 

PUBLISH CAMERA FEED

1. root@jw4g-M16:/catkin_ws/src/# roscd camera_pub/src/

2. chmod +x kitti.py

3. root@jw4g-M16:/catkin_ws/src/camera_pub/src# rosrun camera_pub kitti.py 

4. in another terminal: rviz -> add -> by topic -> kitti/camera_color_left/image_raw -> image  

5. in another terminal: rviz -> add -> by topic -> pcl

6. moving files between host and containers: 
docker cp src/. container_id:/target
docker cp container_id:/src/. target

7. gps data cant be visualize in rviz:
rostopic info /kitti_gps
rostopic echo /kitti_gps
