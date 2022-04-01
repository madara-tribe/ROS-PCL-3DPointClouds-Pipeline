# save bag as image
roslaunch lidar_capture bag2image.launch bag_path:=/home/hagi/place/images/
# check bag file
roscore
rosrun rqt_bag rqt_bag im.bag

# save bag to depth and rgb image
# create pcd from depth and rgb images
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
rosrun image_view image_view image:=/camera/color/image_raw
rosbag record -O /home/parallels/place/depth.bag /camera/color/image_raw /camera/depth/image_rect_raw
python3 bag2depth.py --bag_file /home/hagi/place/images/im.bag
python3 depth2pcd.py
# rosrun pcd_create vccs <pcd_file>
rosrun pcd_create vccs /home/parallels/place/sink_pointcloud.pcd
