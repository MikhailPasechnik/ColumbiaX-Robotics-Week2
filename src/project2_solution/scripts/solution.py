#!/usr/bin/env python
import rospy

import numpy as np

import tf
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Point, Vector3
from tf.transformations import (
    quaternion_from_euler,
    quaternion_from_matrix,
    euler_from_matrix,
    euler_matrix,
    translation_matrix,
    translation_from_matrix,
    compose_matrix,
    quaternion_about_axis,
)

def lookAt(direction, up=(0, 0, 1)):
    dir_x = direction / np.linalg.norm(direction)

    dir_y = np.cross(up, dir_x)
    dir_y = dir_y / np.linalg.norm(dir_y)
    
    dir_z = np.cross(dir_x, dir_y)
    dir_z = dir_z / np.linalg.norm(dir_z)
    
    R = np.c_[dir_x, dir_y, dir_z, [0, 0, 0]]
    R = np.r_[R, [[0, 0, 0, 1]]]
    return R

def publish_transforms():
    T_obj = euler_matrix(0.79, 0.0, 0.79).dot(translation_matrix((0, 1, 1)))
    T_robot = euler_matrix(0, 0.0, 1.5).dot(translation_matrix((0, -1, 0)))

    camera_offset = [0, 0.1, 0.1]

    T_obj_camera = np.linalg.inv(T_robot.dot(translation_matrix(camera_offset))).dot(T_obj)
    R_camera = lookAt(translation_from_matrix(T_obj_camera))
    T_camera = translation_matrix(camera_offset).dot(R_camera)

    obj = geometry_msgs.msg.TransformStamped()
    obj.transform.rotation = Quaternion(*quaternion_from_matrix(T_obj))
    obj.transform.translation = Vector3(*translation_from_matrix(T_obj))
    obj.header.stamp = rospy.Time.now()
    obj.header.frame_id = "base_frame"
    obj.child_frame_id = "object_frame"
    br.sendTransform(obj)
    robot = geometry_msgs.msg.TransformStamped()
    robot.transform.rotation = Quaternion(*quaternion_from_matrix(T_robot))
    robot.transform.translation = Vector3(*translation_from_matrix(T_robot))
    robot.header.stamp = rospy.Time.now()
    robot.header.frame_id = "base_frame"
    robot.child_frame_id = "robot_frame"
    br.sendTransform(robot)
    camera = geometry_msgs.msg.TransformStamped()
    camera.transform.rotation = Quaternion(*quaternion_from_matrix(T_camera))
    camera.transform.translation = Vector3(*translation_from_matrix(T_camera))
    camera.header.stamp = rospy.Time.now()
    camera.header.frame_id = "robot_frame"
    camera.child_frame_id = "camera_frame"
    br.sendTransform(camera)

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
