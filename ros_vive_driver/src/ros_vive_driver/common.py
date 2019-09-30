#!/usr/bin/env python2
import rospy
import tf2_ros
import tf
from tf import transformations as t
from geometry_msgs.msg import Transform, Vector3, Quaternion, TransformStamped

from ros_vive_driver_msgs.msg import ControllerState


def _define_broadcast_static_transform():
    # Only the last transform sent to any StaticTransformBroadcaster is actually latched.
    # To send multiple transforms, you need to put them in a list and send that list with a single call
    # By defining the "broadcast_static_transform" function like this, we don't need to worry about that list,
    # as it is being tracked and updated whenever we want to publish a new static transform using the function
    # https://answers.ros.org/question/261815/how-can-i-access-all-static-tf2-transforms/
    # https://github.com/ros/ros_comm/issues/146

    transforms = []
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    def func(frame_id, child_frame_id, transform):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.seq = len(transforms)
        transform_msg.header.frame_id = frame_id
        transform_msg.child_frame_id = child_frame_id
        transform_msg.transform = transform

        transforms.append(transform_msg)
        broadcaster.sendTransform(transforms)

    return func


broadcast_static_transform = _define_broadcast_static_transform()


def wait_for_message_filtered(topic, msg_type, filter_function, rate=100):
    class WaitForMessageTarget(object):
        def __init__(self, filter_func):
            self.msg = None
            self.filter_func = filter_func

        def cb(self, msg):
            if self.msg is None and filter_function(msg):
                self.msg = msg

    wfm = WaitForMessageTarget(filter_function)
    subscriber = rospy.Subscriber(topic, msg_type, wfm.cb)

    while not rospy.core.is_shutdown():
        if wfm.msg is not None:
            break
        rospy.rostime.wallsleep(1.0 / rate)

    subscriber.unregister()
    if rospy.is_shutdown():
        raise rospy.exceptions.ROSInterruptException("rospy shutdown")

    return wfm.msg


def wait_for_controller_trigger(state_topic):
    wait_for_message_filtered(state_topic, ControllerState, lambda msg: msg.trigger > 0.9)
    wait_for_message_filtered(state_topic, ControllerState, lambda msg: msg.trigger < 0.1)


def keep_waiting_for_transform(listener, source_frame, target_frame):
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform(source_frame, target_frame, rospy.Time(0), rospy.Duration(10))
            return listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
        except tf.Exception:
            pass
    if rospy.is_shutdown():
        raise rospy.exceptions.ROSInterruptException("rospy shutdown")


def pose_to_matrix(pose):
    return t.concatenate_matrices(t.translation_matrix([pose.position.x,
                                                        pose.position.y,
                                                        pose.position.z]),
                                  t.quaternion_matrix([pose.orientation.x,
                                                       pose.orientation.y,
                                                       pose.orientation.z,
                                                       pose.orientation.w]))


def matrix_to_transform(matrix):
    t_list = t.translation_from_matrix(matrix)
    q_list = t.quaternion_from_matrix(matrix)
    translation = Vector3(t_list[0], t_list[1], t_list[2])
    orientation = Quaternion(q_list[0], q_list[1], q_list[2], q_list[3])
    return Transform(translation, orientation)
