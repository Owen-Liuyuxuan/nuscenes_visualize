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
- [ ] Nuscenes sweeps support. (the images / lidar not synchronized well)
- [ ] RADARs.

## Setup

Install nuscenes-devkit
```
pip3 install nuscenes-devkit
```

Install ROS, tested on Ubuntu 20.04, ROS noetic. (noted that the code was first developed on 18.04 with a mix of Python2/Python3). If you want to use this code in Ubuntu 18.04, please try:

```bash
## Checkout to the version with partial python2 support. Codes with nuScenes will run in python3 and codes with tf trees will be in python2. In new version, every thing is in Python3.
git checkout 2bc4abbcb5d8fca5e6b7df57012c6c5a505bab2d

## Enable rospy in Python3 for ROS melodic (this should not affect Python2), but it does not enable tf in Python3.
sudo apt-get install python3-catkin-pkg-modules
sudo apt-get install python3-rospkg-modules
```


Clone this repo into a ROS workspace/src and run
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

