#!/usr/bin/env python3
""" This script is the main node launcher for publishing nuscenes data.

This script has to run in python3, because nuscenes-devkit only runs in python3.6/python3.7.

One important workaround of the current file is to publish pose instead of tf, Because the installation of tf in python3 is difficult and can pollute the system environments.
We take a detour and publish PoseStamped instead, and we add another python2 compatible script 'pose2tf.py' to transfer poses into a tf tree.

To cleanly allow python3 import rospy (this should not affect python2):

```bash
sudo apt-get install python3-catkin-pkg-modules
sudo apt-get install python3-rospkg-modules
```

"""
import os
import rospy 
import numpy as np
import cv2
from nuscenes.nuscenes import NuScenes
from utils import ros_util
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Int32, Bool, Float32
from geometry_msgs.msg import PoseStamped

class NuscenesVisualizationNode:
    def __init__(self):
        rospy.init_node("NuscenesVisualizationNode")
        rospy.loginfo("Starting NuscenesVisualizationNode.")

        self.read_params()
        
        # Most Publishers will be initialized right before the first published data
        self.publishers = {}
        self.publishers["bboxes"] = rospy.Publisher("/nuscenes/bboxes", MarkerArray, queue_size=1, latch=True)

        # This takes some time and block this process.
        self.nusc = NuScenes(version=self.nuscenes_version, dataroot=self.nuscenes_dir, verbose=True)
        print(len(self.nusc.scene)) # 850 with default data

        # Initialize for control status
        self.set_index(0)
        self.pause = False
        self.stop = True
        self.publishing = True
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_frequency), self.publish_callback)
        
        rospy.Subscriber("/nuscenes/control/index", Int32, self.index_callback)
        rospy.Subscriber("/nuscenes/control/stop", Bool, self.stop_callback)
        rospy.Subscriber("/nuscenes/control/pause", Bool, self.pause_callback)
        
    def read_params(self):
        self.nuscenes_dir = rospy.get_param("~NUSCENES_DIR", '/data/nuscene')
        self.nuscenes_version = rospy.get_param("~NUSCENES_VER", 'v1.0-trainval')
        self.update_frequency = float(rospy.get_param("~UPDATE_FREQUENCY", 8.0))

    def set_index(self, index):
        """Set current index -> select scenes -> print(scene description) -> set current sample as the first sample of the scene
        """
        self.index = index
        self.current_scene = self.nusc.scene[self.index]
        des = self.current_scene["description"]
        print(f"Switch to scenes {self.index}: {des} ")
        self.current_sample = self.nusc.get('sample', self.current_scene['first_sample_token']) 

    def stop_callback(self, msg):
        self.published=False
        self.stop = msg.data #Bool
        self.current_sample = self.nusc.get('sample', self.current_scene['first_sample_token']) #back to the first sample
        ros_util.clear_all_bbox(self.publishers['bboxes'])
        self.publish_callback(None)

    def pause_callback(self, msg):
        self.pause = msg.data

    def index_callback(self, msg):
        self.set_index(msg.data)
        self.publish_callback(None)

    def _camera_publish(self, camera_data, is_publish_image=False):
        """Publish camera related data, first publish pose/tf information, then publish image if needed

        Args:
            camera_data (Dict): camera data from nuscenes' sample data
            is_publish_image (bool, optional): determine whether to read image data. Defaults to False.
        """        
        cs_record   = self.nusc.get("calibrated_sensor", camera_data['calibrated_sensor_token'])
        ego_record  = self.nusc.get("ego_pose", camera_data['ego_pose_token'])

        image_path = os.path.join(self.nuscenes_dir, camera_data['filename'])
        imsize    = (camera_data["width"], camera_data["height"])
        channel = camera_data['channel']
        
        # publish relative pose
        cam_intrinsic = np.array(cs_record["camera_intrinsic"]) #[3 * 3]
        rotation = cs_record["rotation"] #list, [4] r, x, y, z
        translation = cs_record['translation'] #
        relative_pose = ros_util.compute_pose(translation, rotation)
        pose_pub_name = f"{channel}_pose_pub"
        if pose_pub_name not in self.publishers:
            self.publishers[pose_pub_name] = rospy.Publisher(f"/nuscenes/{channel}/pose", PoseStamped, queue_size=1, latch=True)
        msg = PoseStamped()
        msg.pose = relative_pose
        msg.header.frame_id = "base_link"
        msg.header.stamp = rospy.Time.now()
        self.publishers[pose_pub_name].publish(msg)

        

        if is_publish_image:
            image_pub_name = f"{channel}_image_pub"
            if image_pub_name not in self.publishers:
                self.publishers[image_pub_name] = rospy.Publisher(f"/nuscenes/{channel}/image", Image, queue_size=1, latch=1)
            info_pub_name  = f"{channel}_info_pub"
            if info_pub_name not in self.publishers:
                self.publishers[info_pub_name] = rospy.Publisher(f"/nuscenes/{channel}/camera_info", CameraInfo, queue_size=1, latch=1)

            image = cv2.imread(image_path)
            ros_util.publish_image(image,
                                   self.publishers[image_pub_name],
                                   self.publishers[info_pub_name],
                                   cam_intrinsic,
                                   channel)

    def _lidar_publish(self, lidar_data, is_publish_lidar=False):
        """Publish lidar related data, first publish pose/tf information and ego pose, then publish lidar if needed

        Args:
            lidar_data (Dict): lidar data from nuscenes' sample data
            is_publish_lidar (bool, optional): determine whether to read lidar data. Defaults to False.
        """
        cs_record   = self.nusc.get("calibrated_sensor", lidar_data['calibrated_sensor_token'])
        ego_record  = self.nusc.get("ego_pose", lidar_data['ego_pose_token'])

        lidar_path = os.path.join(self.nuscenes_dir, lidar_data['filename'])
        channel = 'LIDAR_TOP'
        
        # publish relative pose
        cam_intrinsic = np.array(cs_record["camera_intrinsic"]) #[3 * 3]
        rotation = cs_record["rotation"] #list, [4] r, x, y, z
        translation = cs_record['translation'] #
        relative_pose = ros_util.compute_pose(translation, rotation)
        pose_pub_name = f"{channel}_pose_pub"
        if pose_pub_name not in self.publishers:
            self.publishers[pose_pub_name] = rospy.Publisher(f"/nuscenes/{channel}/pose", PoseStamped, queue_size=1, latch=True)
        msg = PoseStamped()
        msg.pose = relative_pose
        msg.header.frame_id = "base_link"
        msg.header.stamp = rospy.Time.now()
        self.publishers[pose_pub_name].publish(msg)

        # publish ego pose
        ego_rotation = ego_record["rotation"]
        ego_translation = ego_record["translation"]
        frame_location = ros_util.compute_pose(ego_translation, ego_rotation)
        pose_pub_name = "ego_pose"
        
        if pose_pub_name not in self.publishers:
            self.publishers[pose_pub_name] = rospy.Publisher("/nuscenes/ego_pose", PoseStamped, queue_size=1, latch=True)
        msg = PoseStamped()
        msg.pose = frame_location
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        self.publishers[pose_pub_name].publish(msg)

        # publish lidar
        if is_publish_lidar:
            point_cloud = np.fromfile(os.path.join(self.nuscenes_dir, lidar_data['filename']), dtype=np.float32).reshape(-1, 5)[:, :4]
            lidar_pub_name = 'lidar_pub'
            if lidar_pub_name not in self.publishers:
                self.publishers[lidar_pub_name] = rospy.Publisher("/nuscenes/LIDAR_TOP/data", PointCloud2, queue_size=1, latch=True)

            ros_util.publish_point_cloud(point_cloud, self.publishers[lidar_pub_name], "LIDAR_TOP")
              
    def publish_callback(self, event):
        if self.stop: # if stopped, falls back to an empty loop
            return

        # Publish cameras and camera info
        channels = ['CAM_BACK',  'CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK_RIGHT', 'CAM_BACK_LEFT']
        for channel in channels:
            data = self.nusc.get('sample_data', self.current_sample['data'][channel])
            self._camera_publish(data, is_publish_image=self.publishing)

        # Publish lidar and ego pose
        data = self.nusc.get('sample_data', self.current_sample['data']['LIDAR_TOP'])
        self._lidar_publish(data, self.publishing)

        # Publish 3D bounding boxes
        _, bboxes, _ = self.nusc.get_sample_data(self.current_sample['data']['LIDAR_TOP'])
        markers = [ros_util.object_to_marker(box, frame_id='LIDAR_TOP', marker_id=i, duration= 1.2 / self.update_frequency) for i, box in enumerate(bboxes)]
        self.publishers['bboxes'].publish(markers)

        self.publishing = not self.pause # if paused, the original images and lidar are latched (as defined in publishers) and we will not re-publish them to save memory access. But we need to re-publish tf and markers

        if not self.pause:
            if (self.current_sample['next'] == ''):
                # If end reached, loop back from the start
                self.current_sample = self.nusc.get('sample', self.current_scene['first_sample_token']) 
            else:
                self.current_sample = self.nusc.get('sample', self.current_sample['next'])


if __name__ == "__main__":
    ros_node = NuscenesVisualizationNode()
    rospy.spin()