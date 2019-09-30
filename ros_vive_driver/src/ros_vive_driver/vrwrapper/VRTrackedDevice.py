import math

import openvr


class VRTrackedDevice(object):
    TRACKING_RESULT = {
        1: "Uninitialized",
        100: "Calibrating_InProgress",
        101: "Calibrating_OutOfRange",
        200: "Running_OK",
        201: "Running_OutOfRange"
    }

    def __init__(self, vr_obj, index, device_class, name):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj
        self.pose_cache = openvr.TrackedDevicePose_t()
        self.name = name

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        pass

    def get_serial(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_SerialNumber_String).decode('utf-8')

    def get_model(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModelNumber_String).decode('utf-8')

    def get_pose(self, force_update=False):
        pose = self.pose_cache
        if force_update:
            pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                           openvr.k_unMaxTrackedDeviceCount)[self.index]
        if not pose.bPoseIsValid:
            return None
        return VRTrackedDevice.convert_matrix_to_pose(pose.mDeviceToAbsoluteTracking)

    def get_velocity(self, force_update=False):
        pose = self.pose_cache
        if force_update:
            pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                           openvr.k_unMaxTrackedDeviceCount)[self.index]
        if not pose.bPoseIsValid:
            return None
        velocity = pose.vVelocity
        angular_velocity = pose.vAngularVelocity
        return (velocity[0], velocity[1], velocity[2]), (angular_velocity[0], angular_velocity[1], angular_velocity[2])

    def has_valid_pose(self, force_update=False):
        pose = self.pose_cache
        if force_update:
            pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                           openvr.k_unMaxTrackedDeviceCount)[self.index]
        return pose.bPoseIsValid == 1

    def is_connected(self, force_update=False):
        pose = self.pose_cache
        if force_update:
            pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                           openvr.k_unMaxTrackedDeviceCount)[self.index]
        return pose.bDeviceIsConnected == 1

    def get_tracking_result(self, force_update=False):
        pose = self.pose_cache
        if force_update:
            pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                           openvr.k_unMaxTrackedDeviceCount)[self.index]
        if pose.eTrackingResult not in VRTrackedDevice.TRACKING_RESULT:
            return "Unknown"
        return VRTrackedDevice.TRACKING_RESULT[pose.eTrackingResult]

    def update_pose(self, poses):
        self.pose_cache = poses[self.index]

    @staticmethod
    def convert_matrix_to_pose(pose_mat):
        # Changed from triad_openvr version since that one could crash due to divide by 0 error.
        # This calculation comes from issue #3
        r_w = math.sqrt(max(0, 1 + pose_mat[0][0] + pose_mat[1][1] + pose_mat[2][2])) * 0.5
        r_x = math.sqrt(max(0, 1 + pose_mat[0][0] - pose_mat[1][1] - pose_mat[2][2])) * 0.5
        r_y = math.sqrt(max(0, 1 - pose_mat[0][0] + pose_mat[1][1] - pose_mat[2][2])) * 0.5
        r_z = math.sqrt(max(0, 1 - pose_mat[0][0] - pose_mat[1][1] + pose_mat[2][2])) * 0.5

        r_x *= math.copysign(1, r_x * (pose_mat[2][1] - pose_mat[1][2]))
        r_y *= math.copysign(1, r_y * (pose_mat[0][2] - pose_mat[2][0]))
        r_z *= math.copysign(1, r_z * (pose_mat[1][0] - pose_mat[0][1]))

        x = pose_mat[0][3]
        y = pose_mat[1][3]
        z = pose_mat[2][3]

        return (x, y, z), (r_x, r_y, r_z, r_w)

