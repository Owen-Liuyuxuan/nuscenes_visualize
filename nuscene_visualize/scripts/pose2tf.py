#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import PoseStamped
import geometry_msgs
from std_msgs.msg import Int32
import tf
"""
    Current tf tree for nuscenes:
        world -> odom (first pose for each sequence)
        world -> base_link -> all_other_sensors
"""
class Pose2tfNode:
    def __init__(self):
        rospy.init_node("Pose2tfNode")
        rospy.loginfo("Starting Pose2tfNode.")

        self.initial_pose = None
        rospy.Subscriber("/nuscenes/CAM_FRONT/pose", PoseStamped, self.FRONT_pose_cb)
        rospy.Subscriber("/nuscenes/CAM_FRONT_LEFT/pose", PoseStamped, self.FRONT_LEFT_pose_cb)
        rospy.Subscriber("/nuscenes/CAM_FRONT_RIGHT/pose", PoseStamped, self.FRONT_RIGHT_pose_cb)
        rospy.Subscriber("/nuscenes/CAM_BACK/pose", PoseStamped, self.BACK_pose_cb)
        rospy.Subscriber("/nuscenes/CAM_BACK_LEFT/pose", PoseStamped, self.BACK_LEFT_pose_cb)
        rospy.Subscriber("/nuscenes/CAM_BACK_RIGHT/pose", PoseStamped, self.BACK_RIGHT_pose_cb)
        rospy.Subscriber("/nuscenes/LIDAR_TOP/pose", PoseStamped, self.LIDAR_TOP_pose_cb)
        rospy.Subscriber("/nuscenes/ego_pose", PoseStamped, self.ego_pose_cb)

        rospy.Subscriber("/nuscenes/control/index", Int32, self.index_callback) # to clean up initial pose

    def index_callback(self, msg):
        self.initial_pose = None # reset odom

    def publish_tf(self, pose_stamped, current_frame, frame_id = None, new_stamp=False):
        assert isinstance(pose_stamped, PoseStamped)
        br = tf.TransformBroadcaster()

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = pose_stamped.header.frame_id if frame_id is None else frame_id
        t.header.stamp = rospy.Time.now() if new_stamp else pose_stamped.header.stamp
        t.child_frame_id = current_frame
        t.transform.translation = pose_stamped.pose.position
        t.transform.rotation    = pose_stamped.pose.orientation

        br.sendTransformMessage(t)
        
    def FRONT_pose_cb(self, msg):
        self.publish_tf(msg, 'CAM_FRONT')

    def FRONT_LEFT_pose_cb(self, msg):
        self.publish_tf(msg, 'CAM_FRONT_LEFT')

    def FRONT_RIGHT_pose_cb(self, msg):
        self.publish_tf(msg, 'CAM_FRONT_RIGHT')

    def BACK_pose_cb(self, msg):
        self.publish_tf(msg, 'CAM_BACK')
    
    def BACK_LEFT_pose_cb(self, msg):
        self.publish_tf(msg, 'CAM_BACK_LEFT')
    
    def BACK_RIGHT_pose_cb(self, msg):
        self.publish_tf(msg, 'CAM_BACK_RIGHT')

    def LIDAR_TOP_pose_cb(self, msg):
        self.publish_tf(msg, 'LIDAR_TOP')

    def ego_pose_cb(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg
        self.publish_tf(self.initial_pose, 'odom', new_stamp=True)
        self.publish_tf(msg, 'base_link')

if __name__ == "__main__":
    ros_node = Pose2tfNode()
    rospy.spin()