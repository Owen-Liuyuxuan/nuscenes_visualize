#!/usr/bin/env python
import rospy 
import numpy as np
from math import sin, cos
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Int32, Bool
from cv_bridge import CvBridge
import cv2
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
from .constants import general_to_detection, nuscenes_names, colors
from geometry_msgs.msg import Pose

def compute_pose(translation, rotation):
    """Compute Pose from translation and rotation

    Args:
        translation (List[float]): x, y, z
        rotation (List[float]): w, x, y, z

    Returns:
        geometry_msgs.msg.Pose
    """    
    msg = Pose()
    msg.orientation.w = rotation[0]
    msg.orientation.x = rotation[1]
    msg.orientation.y = rotation[2]
    msg.orientation.z = rotation[3]

    msg.position.x    = translation[0]
    msg.position.y    = translation[1]
    msg.position.z    = translation[2]
    return msg

def depth_image_to_point_cloud_array(depth_image, K, parent_frame="left_camera", rgb_image=None):
    """  convert depth image into color pointclouds [xyzbgr]
    
    """
    depth_image = np.copy(depth_image) / 256.0
    w_range = np.arange(0, depth_image.shape[1], dtype=np.float32)
    h_range = np.arange(0, depth_image.shape[0], dtype=np.float32)
    w_grid, h_grid = np.meshgrid(w_range, h_range) #[H, W]
    K_expand = np.eye(4)
    K_expand[0:3, 0:3] = K
    K_inv = np.linalg.inv(K_expand) #[4, 4]

    #[H, W, 4, 1]
    expand_image = np.stack([w_grid * depth_image, h_grid * depth_image, depth_image, np.ones_like(depth_image)], axis=2)[...,np.newaxis]

    pc_3d = np.matmul(K_inv, expand_image)[..., 0:3, 0] #[H, W, 3]
    if rgb_image is not None:
        pc_3d = np.concatenate([pc_3d, rgb_image/256.0], axis=2).astype(np.float32)
    point_cloud = pc_3d[depth_image > 0,:]
    
    return point_cloud

def line_points_from_3d_bbox(x, y, z, w, h, l, theta):
    """Compute line points for Rviz Marker from 3D bounding box data

    Args:
        x (float): 
        y (float):
        z (float): 
        w (float): 
        h (float): 
        l (float): 
        theta (float): angular rotation around z axis

    Returns:
        List[Point] : directly usable for Lines Marker
    """    
    corner_matrix = np.array(
        [[-1, -1, -1],
        [ 1, -1, -1],
        [ 1,  1, -1],
        [ 1,  1,  1],
        [ 1, -1,  1],
        [-1, -1,  1],
        [-1,  1,  1],
        [-1,  1, -1]], dtype=np.float32
    )
    relative_eight_corners = 0.5 * corner_matrix * np.array([l, w, h]) #[8, 3]

    _cos = cos(theta)
    _sin = sin(theta)

    rotated_corners_x, rotated_corners_y = (
            relative_eight_corners[:, 0] * _cos +
                -relative_eight_corners[:, 1] * _sin,
        relative_eight_corners[:, 0] * _sin +
            relative_eight_corners[:, 1] * _cos
        ) #[8]
    rotated_corners = np.stack([rotated_corners_x, rotated_corners_y, relative_eight_corners[:,2]], axis=-1) #[8, 3]
    abs_corners = rotated_corners + np.array([x, y, z])  # [8, 3]

    points = []
    for i in range(1, 5):
        points += [
            Point(x=abs_corners[i, 0], y=abs_corners[i, 1], z=abs_corners[i, 2]),
            Point(x=abs_corners[i%4+1, 0], y=abs_corners[i%4+1, 1], z=abs_corners[i%4+1, 2])
        ]
        points += [
            Point(x=abs_corners[(i + 4)%8, 0], y=abs_corners[(i + 4)%8, 1], z=abs_corners[(i + 4)%8, 2]),
            Point(x=abs_corners[((i)%4 + 5)%8, 0], y=abs_corners[((i)%4 + 5)%8, 1], z=abs_corners[((i)%4 + 5)%8, 2])
        ]
    points += [
        Point(x=abs_corners[2, 0], y=abs_corners[2, 1], z=abs_corners[2, 2]),
        Point(x=abs_corners[7, 0], y=abs_corners[7, 1], z=abs_corners[7, 2]),
        Point(x=abs_corners[3, 0], y=abs_corners[3, 1], z=abs_corners[3, 2]),
        Point(x=abs_corners[6, 0], y=abs_corners[6, 1], z=abs_corners[6, 2]),

        Point(x=abs_corners[4, 0], y=abs_corners[4, 1], z=abs_corners[4, 2]),
        Point(x=abs_corners[5, 0], y=abs_corners[5, 1], z=abs_corners[5, 2]),
        Point(x=abs_corners[0, 0], y=abs_corners[0, 1], z=abs_corners[0, 2]),
        Point(x=abs_corners[1, 0], y=abs_corners[1, 1], z=abs_corners[1, 2])
    ]

    return points

def object_to_marker(box, frame_id="base", marker_id=None, duration=0.15, color=None):
    """ Transform an object to a marker.

    Args:
        box: Nuscene.dataclasses.Box
        frame_id: string; parent frame name
        marker_id: visualization_msgs.msg.Marker.id
        duration: the existence time in rviz
    
    Return:
        marker: visualization_msgs.msg.Marker

    To visualize 3D bounding boxes, we apply the line list type in http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html.
    Line list will create a array of line formed by every two points in the marker.points list.
    """
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = frame_id
    if marker_id is not None:
        marker.id = marker_id
    marker.type = 5 
    marker.scale.x = 0.05

    detection_name = general_to_detection[box.name]
    if color is None:
        obj_color = colors[detection_name] #[r, g, b]
    else:
        obj_color = color
    marker.color.r = obj_color[0] / 255.0
    marker.color.g = obj_color[1] / 255.0
    marker.color.b = obj_color[2] / 255.0
    marker.color.a = 1.0 if np.isnan(box.score) else float(box.score)
    marker.points = line_points_from_3d_bbox(
                        box.center[0], box.center[1], box.center[2],
                        box.wlh[0]   , box.wlh[2]   , box.wlh[1]   , box.orientation.yaw_pitch_roll[0])
    marker.lifetime = rospy.Duration.from_sec(duration)
    return marker

def publish_image(image, image_publisher, camera_info_publisher, P, frame_id):
    """Publish image and info message to ROS.

    Args:
        image: numpy.ndArray.
        image_publisher: rospy.Publisher
        camera_info_publisher: rospy.Publisher, should publish CameraInfo
        P: projection matrix [3, 4]. though only [3, 3] is useful.
        frame_id: string, parent frame name.
    """
    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    image_msg.header.frame_id = frame_id
    image_msg.header.stamp = rospy.Time.now()
    image_publisher.publish(image_msg)

    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.header.stamp = rospy.Time.now()
    camera_info_msg.height = image.shape[0]
    camera_info_msg.width = image.shape[1]
    camera_info_msg.D = [0, 0, 0, 0, 0]
    camera_info_msg.K = np.reshape(P[0:3, 0:3], (-1)).tolist()
    P_no_translation = np.zeros([3, 4])
    P_no_translation[0:3, 0:3] = P[0:3, 0:3]
    camera_info_msg.P = np.reshape(P_no_translation, (-1)).tolist()

    camera_info_publisher.publish(camera_info_msg)

def array2pc2(points, parent_frame, field_names='xyza'):
    """ Creates a point cloud message.
    Args:
        points: Nxk array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
        field_names : name for the k channels repectively i.e. "xyz" / "xyza"
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate(field_names)]

    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * len(field_names)),
        row_step=(itemsize * len(field_names) * points.shape[0]),
        data=data
    )

def publish_point_cloud(pointcloud, pc_publisher, frame_id, field_names='xyza'):
    """Convert point cloud array to PointCloud2 message and publish
    
    Args:
        pointcloud: point cloud array [N,3]/[N,4]
        pc_publisher: ROS publisher for PointCloud2
        frame_id: parent_frame name.
        field_names: name for each channel, ['xyz', 'xyza'...]
    """
    msg = array2pc2(pointcloud, frame_id, field_names)
    pc_publisher.publish(msg)

def clear_all_bbox(marker_publisher):
    """ Clear all bboxes published from the current marker publisher.
    Args:
        marker_publisher (rospy.Publisher): publishers on Marker or Marker Array
    """
    clear_marker = Marker()
    clear_marker.action = 3
    if marker_publisher.data_class is Marker:
        marker_publisher.publish(clear_marker)
        return
    if marker_publisher.data_class is MarkerArray:
        marker_publisher.publish([clear_marker])
