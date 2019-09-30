#!/usr/bin/env python2

import openvr

import rospy
import tf2_ros
from tf import transformations as t
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Point, Quaternion, TwistStamped, Vector3
from std_msgs.msg import Bool, String

from ros_vive_driver_msgs.msg import ControllerState
from ros_vive_driver_msgs.srv import VibrateControllerContinuous, VibrateControllerPulse

from vrwrapper import VRWrapper
from common import broadcast_static_transform
from vive_orientation_corrections import correct_orientation


def main():
    rospy.init_node('vive_device_publisher')
    vive_frame = rospy.get_param("~vive_frame")
    topic_prefix = rospy.get_param("~topic_prefix", "/vive")

    try:
        vr = VRWrapper.VRWrapper()
    except openvr.OpenVRError:
        rospy.logfatal("Could not start OpenVR. Is SteamVR running?")
        return

    vibrate_continuous_service = rospy.Service(topic_prefix + "/vibrate_controller_continuous",
                                               VibrateControllerContinuous,
                                               lambda req: vibrate_controller_continuous(vr, req))

    vibrate_pulse_service = rospy.Service(topic_prefix + "/vibrate_controller_pulse",
                                          VibrateControllerPulse,
                                          lambda req: vibrate_controller_pulse(vr, req))

    broadcaster = tf2_ros.TransformBroadcaster()
    device_pose_publishers = {}
    device_twist_publishers = {}
    device_connected_publishers = {}
    device_tracking_result_publishers = {}
    device_valid_pose_publishers = {}
    controller_state_publishers = {}
    broadcasted_tracking_references = set()
    try:
        while not rospy.is_shutdown():
            vr.find_new_devices()
            vr.wait_update_poses()

            for device in vr.get_devices():
                full_device_name = topic_prefix + "/" + device.name
                if device not in vr.get_tracking_references():
                    transform_msg = publish_device_frame(device, broadcaster, full_device_name, vive_frame)
                    publish_device_pose(device, device_pose_publishers, full_device_name, vive_frame)
                    publish_device_twist(device, device_twist_publishers, full_device_name, transform_msg)

                    publish_is_connected(device, device_connected_publishers, full_device_name)
                    publish_is_valid_pose(device, device_valid_pose_publishers, full_device_name)
                    publish_tracking_result(device, device_tracking_result_publishers, full_device_name)

                if device in vr.get_controllers():
                    publish_controller_state(device, controller_state_publishers, full_device_name)

                if device in vr.get_tracking_references():
                    if device not in broadcasted_tracking_references:
                        publish_tracking_reference_static_frame(device, broadcasted_tracking_references,
                                                                full_device_name, vive_frame)

    finally:
        vibrate_continuous_service.shutdown()
        vibrate_pulse_service.shutdown()
        vr.shutdown()


def publish_device_frame(device, broadcaster, full_device_name, frame):
    # Only publish if a pose is available
    device_pose = device.get_pose()
    if device_pose is None:
        return
    position, orientation = device_pose
    orientation = correct_orientation(device, orientation)

    # Publish the device frame
    transform_msg = TransformStamped()
    transform_msg.header.frame_id = frame
    transform_msg.header.stamp = rospy.Time.now()
    transform_msg.child_frame_id = full_device_name
    transform_msg.transform.translation = Point(*position)
    transform_msg.transform.rotation = Quaternion(*orientation)
    broadcaster.sendTransform(transform_msg)
    return transform_msg


def publish_is_connected(device, device_connected_publishers, full_device_name):
    # Get the device publisher, or make a new one if it doesn't exist yet
    if device.name not in device_connected_publishers:
        device_connected_publishers[device.name] = rospy.Publisher(full_device_name + "/connected", Bool, queue_size=1)
    publisher = device_connected_publishers[device.name]
    publisher.publish(Bool(device.is_connected()))


def publish_is_valid_pose(device, device_valid_pose_publishers, full_device_name):
    # Get the device publisher, or make a new one if it doesn't exist yet
    if device.name not in device_valid_pose_publishers:
        device_valid_pose_publishers[device.name] = rospy.Publisher(full_device_name + "/is_tracking", Bool, queue_size=1)
    publisher = device_valid_pose_publishers[device.name]
    publisher.publish(Bool(device.has_valid_pose()))


def publish_tracking_result(device, device_tracking_result_publishers, full_device_name):
    # Get the device publisher, or make a new one if it doesn't exist yet
    if device.name not in device_tracking_result_publishers:
        device_tracking_result_publishers[device.name] = rospy.Publisher(full_device_name + "/tracking_result", String, queue_size=1)
    publisher = device_tracking_result_publishers[device.name]
    publisher.publish(String(device.get_tracking_result()))


def publish_device_pose(device, device_pose_publishers, full_device_name, frame):
    # Get the device pose publisher, or make a new one if it doesn't exist yet
    if device.name not in device_pose_publishers:
        rospy.loginfo("Discovered VR Device: %s (Serial: %s)" % (device.name, device.get_serial()))
        device_pose_publishers[device.name] = rospy.Publisher(full_device_name, PoseStamped,
                                                              queue_size=1)
    publisher = device_pose_publishers[device.name]

    # Only publish if a pose is available
    device_pose = device.get_pose()
    if device_pose is None:
        return
    position, orientation = device_pose
    orientation = correct_orientation(device, orientation)

    # Publish a Pose message
    pose = PoseStamped()
    pose.pose.position = Point(*position)
    pose.pose.orientation = Quaternion(*orientation)
    pose.header.frame_id = frame
    pose.header.stamp = rospy.Time.now()
    publisher.publish(pose)


def publish_device_twist(device, device_twist_publishers, full_device_name, transform_msg):
    # Get the device twist publisher, or make a new one if it doesn't exist yet
    if device.name not in device_twist_publishers:
        device_twist_publishers[device.name] = rospy.Publisher(full_device_name + "/twist", TwistStamped, queue_size=1)
    publisher = device_twist_publishers[device.name]

    # Only publish if a pose is available
    device_velocity = device.get_velocity()
    if device_velocity is None:
        return
    linear, angular = device_velocity

    # Have to do the transformations manually, since tf may not be fully updated yet at this point

    # Measurements are made in the vive frame
    # --> We need to cancel out the rotation from the vive frame to the device frame
    transform_matrix = t.inverse_matrix(
        t.quaternion_matrix([transform_msg.transform.rotation.x,
                             transform_msg.transform.rotation.y,
                             transform_msg.transform.rotation.z,
                             transform_msg.transform.rotation.w]))

    # Both linear and angular velocity are vectors, so we can treat them like translations
    linear_vel_matrix = t.translation_matrix(linear)
    angular_vel_matrix = t.translation_matrix(angular)

    # Concatenating the transformation matrices rotates the velocity vectors to the device frame
    transformed_linear_vel_matrix = t.concatenate_matrices(transform_matrix, linear_vel_matrix)
    transformed_angular_vel_matrix = t.concatenate_matrices(transform_matrix, angular_vel_matrix)

    twist = TwistStamped()
    twist.header.frame_id = full_device_name
    twist.header.stamp = rospy.Time.now()
    twist.twist.linear = Vector3(*t.translation_from_matrix(transformed_linear_vel_matrix))
    twist.twist.angular = Vector3(*t.translation_from_matrix(transformed_angular_vel_matrix))
    publisher.publish(twist)


def publish_controller_state(controller, controller_state_publishers, full_device_name):
    # Get the controller state publisher, or make a new one if it doesn't exist yet
    if controller.name not in controller_state_publishers:
        controller_state_publishers[controller.name] = rospy.Publisher(
            full_device_name + '/state',
            ControllerState, queue_size=1)
    publisher = controller_state_publishers[controller.name]

    # Get the controller state, create the ControllerState message and publish it
    state = controller.get_controller_state()
    state_msg = ControllerState()
    state_msg.controller_name = controller.name
    state_msg.trigger = state.trigger
    state_msg.menu_button = state.menu_button
    state_msg.grip_button = state.grip_button
    state_msg.trackpad_x = state.trackpad_x
    state_msg.trackpad_y = state.trackpad_y
    state_msg.trackpad_touched = state.trackpad_touched
    state_msg.trackpad_pressed = state.trackpad_pressed
    publisher.publish(state_msg)


def publish_tracking_reference_static_frame(device, broadcasted_tracking_references, full_device_name, frame):
    if device in broadcasted_tracking_references:
        return

    pose = device.get_pose()
    if pose is None:
        return
    broadcasted_tracking_references.add(device)
    rospy.loginfo("Discovered Base Station: %s (Serial: %s)" % (device.name, device.get_serial()))

    position, orientation = device.get_pose()
    broadcast_static_transform(frame, full_device_name,
                               Transform(Point(*position), Quaternion(*orientation)))


def vibrate_controller_continuous(vr, req):
    if req.name not in vr.object_names["Controller"]:
        return False
    controller = vr.devices[req.name]
    controller.start_continuous_vibration(req.strength)
    return True


def vibrate_controller_pulse(vr, req):
    if req.name not in vr.object_names["Controller"]:
        return False
    controller = vr.devices[req.name]
    controller.start_multiple_vibration_pulses(req.strength, req.pulse_duration, req.pulse_count, req.pulse_period)
    return True


if __name__ == '__main__':
    main()
