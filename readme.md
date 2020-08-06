# Nuscenes Visualization with ROS-Rviz

This package aims to visualize [nuscenes](https://www.nuscenes.org) dataset in rviz environment. 

We fully export nuscenes data to standard ROS topics.

## Visualizing for Nuscenes in ROS
```
# modify path in launch file as necessary
roslaunch nuscene_visualize visualize_launch.launch 
```
![gif](docs/nuscene_visualized.gif)

## Key features

- [x] Nuscenes keyframes support. 
- [x] Six cameras.
- [x] LiDARs.
- [x] Ground truth bounding boxes.
- [x] Pose and TF-trees (world, odom, base_link and sensors).
- [x] GUI control & ROS topic control.
- [ ] Nuscenes sweeps support.
- [ ] RADARs.

## Setup

Install ROS, tested on Ubuntu 18.04, ROS melodic.

Install nuscenes-devkit
```
pip3 install nuscenes-devkit
```

Enable rospy in Python3 (this should not affect Python2), but it **does not** enable **tf** in Python3.
```bash
sudo apt-get install python3-catkin-pkg-modules
sudo apt-get install python3-rospkg-modules
```

Clone this repo into a ROS workspace and run
```bash
catkin_make
source devel/setup.bash
```
under the workspace folder.

## Default Topics

### Publish

| topic name                        | Description |
|-----------------------------------|---------------|
| /nuscenes/{channel}/pose          | sensor coordinates w.r.t. to base_link|
| /nuscenes/{channel}/image         | image.|
| /nuscenes/{channel}/camera_info   | intrinsic parameters.|
| /nuscenes/ego_pose                | Relative pose base_link w.r.t to the world.|
| /nuscenes/LIDAR_TOP/data          | Point cloud data.|
| /nuscenes/bboxes                  | MarkerArray visualizing bounding boxes.

