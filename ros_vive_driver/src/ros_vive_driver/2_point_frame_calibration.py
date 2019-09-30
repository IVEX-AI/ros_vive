#!/usr/bin/env python2

import math

import rospy
import tf
from tf import transformations as t

from geometry_msgs.msg import Pose, Point, Quaternion

from ros_vive_driver_msgs.srv import VibrateControllerPulse

import common


def main():
    rospy.init_node('vive_2_point_frame_calibration')

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

    transform_1_list = common.keep_waiting_for_transform(listener, vive_frame, pose_frame)
    pose1 = Pose(Point(*transform_1_list[0]), Quaternion(*transform_1_list[1]))
    rospy.loginfo(
        "Got valid first pose:\n%s, \n%s" % (str(pose1.position), str(pose1.orientation)))

    if not use_controller:
        raw_input("Please move the tracker to the right along the X axis and press enter")
    else:
        print("Please move the tracker to the right along the X axis and pull the trigger on the controller")
        vibrate(controller_name, 1, 0.05, 5, 0.1)
        common.wait_for_controller_trigger(controller_state_topic)

    rospy.loginfo("Waiting for a valid second pose...")
    transform_2_list = common.keep_waiting_for_transform(listener, vive_frame, pose_frame)
    pose2 = Pose(Point(*transform_2_list[0]), Quaternion(*transform_2_list[1]))
    rospy.loginfo(
        "Got valid second pose:\n%s, \n%s" % (str(pose2.position), str(pose2.orientation)))

    # Convert the two measured tracker poses into transformation matrices in the vive_world frame
    p1_mtx = common.pose_to_matrix(pose1)
    p2_mtx = common.pose_to_matrix(pose2)

    # Tracker orientation by default is upside down (z points downwards). roll by 180 degrees to make z point upwards
    # correction_mtx = t.euler_matrix(math.pi, 0, 0)

    # Construct the initial transformation matrix from world to vive_world.
    # This transformation correctly sets the position, roll and pitch, but the yaw is not correct yet
    # initial_transform_mtx = t.inverse_matrix(t.concatenate_matrices(p1_mtx, correction_mtx))
    initial_transform_mtx = t.inverse_matrix(p1_mtx)

    # Transform the poses in vive_world frame to world frame, using the initial transformation matrix
    p1_trans_list = t.translation_from_matrix(t.concatenate_matrices(initial_transform_mtx, p1_mtx))
    p2_trans_list = t.translation_from_matrix(t.concatenate_matrices(initial_transform_mtx, p2_mtx))

    # Given the 2 poses in world frame, calculate the polar angle of the second pose.
    angle = math.atan2(p2_trans_list[1] - p1_trans_list[1], p2_trans_list[0] - p1_trans_list[0])

    # The final transformation matrix the same as the initial one,
    # with a correction for the yaw using the angle calculated above
    # transform_mtx = t.inverse_matrix(t.concatenate_matrices(p1_mtx, correction_mtx, t.euler_matrix(0, 0, angle)))
    transform_mtx = t.inverse_matrix(t.concatenate_matrices(p1_mtx, t.euler_matrix(0, 0, angle)))

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
