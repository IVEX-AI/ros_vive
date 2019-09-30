#!/usr/bin/env python2

import math

import rospy
import tf
from tf import transformations as t

from geometry_msgs.msg import Pose, Point, Quaternion

from ros_vive_driver_msgs.srv import VibrateControllerPulse

import common


def main():
    rospy.init_node('vive_1_point_frame_calibration')

    vive_frame = rospy.get_param("~vive_frame")
    tracker_name = rospy.get_param("~tracker_name")
    target_frame = rospy.get_param("~target_frame")
    use_controller = rospy.get_param("~use_controller")
    controller_name = "null_controller"
    if use_controller:
        controller_name = rospy.get_param("~controller_name")
    topic_prefix = rospy.get_param("~topic_prefix", "/vive")

    pose_frame = "%s/%s" % (topic_prefix, tracker_name)
    controller_state_topic = "%s/%s/state" % (topic_prefix, controller_name)

    def vibrate():
        pass  # Placeholder

    if use_controller:
        print("Waiting for controller vibration service...")
        rospy.wait_for_service('%s/vibrate_controller_pulse' % topic_prefix)
        vibrate = rospy.ServiceProxy('%s/vibrate_controller_pulse' % topic_prefix, VibrateControllerPulse)

    listener = tf.TransformListener()

    if use_controller:
        print("Please pull the trigger on the controller to measure the tracker's position")
        vibrate(controller_name, 1, 0.05, 5, 0.1)
        common.wait_for_controller_trigger(controller_state_topic)

    rospy.loginfo("Waiting for a valid first pose...")

    transform_list = common.keep_waiting_for_transform(listener, vive_frame, pose_frame)
    pose = Pose(Point(*transform_list[0]), Quaternion(*transform_list[1]))
    rospy.loginfo(
        "Got valid first pose:\n%s, \n%s" % (str(pose.position), str(pose.orientation)))

    # Convert the measured tracker pose into transformation matrix in the vive_world frame
    pose_mtx = common.pose_to_matrix(pose)

    # Tracker orientation by default is upside down (z points downwards). roll by 180 degrees to make z point upwards
    # correction_mtx = t.euler_matrix(math.pi, 0, 0)

    # Construct the transformation matrix from world to vive_world
    # transform_mtx = t.inverse_matrix(t.concatenate_matrices(pose_mtx, correction_mtx))
    transform_mtx = t.inverse_matrix(pose_mtx)

    transform_msg = common.matrix_to_transform(transform_mtx)

    common.broadcast_static_transform(target_frame, vive_frame, transform_msg)
    rospy.loginfo("Translation: " + str(transform_msg.translation))
    rospy.loginfo("Rotation: " + str(transform_msg.rotation))

    if use_controller:
        vibrate(controller_name, 0.5, 0.5, 2, 1)

    rospy.loginfo("Frame calibration completed!")
    rospy.spin()


if __name__ == '__main__':
    main()
