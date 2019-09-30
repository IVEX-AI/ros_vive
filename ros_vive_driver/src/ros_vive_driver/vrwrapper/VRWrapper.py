import openvr

from .VRTrackedDevice import VRTrackedDevice
from .VRTrackingReference import VRTrackingReference
from .ViveController import ViveController


class VRWrapper:
    def __init__(self):
        self.is_shutdown = False
        self.device_serials = set()
        self.object_names = {"Tracking Reference": [], "HMD": [], "Controller": [], "Tracker": []}
        self.devices = {}
        self.vr = openvr.init(openvr.VRApplication_Scene)
        self.find_new_devices()

    def __del__(self):
        if not self.is_shutdown:
            self.shutdown()

    def shutdown(self):
        for device in self.devices.itervalues():
            device.shutdown()
        openvr.shutdown()
        self.is_shutdown = True

    def find_new_devices(self):
        if self.is_shutdown:
            return
        for i in range(openvr.k_unMaxTrackedDeviceCount):

            # Only consider connected devices
            if not self.vr.isTrackedDeviceConnected(i):
                continue

            # Only consider devices which haven't been added yet
            if self.get_serial(i) in self.device_serials:
                continue

            device_class = self.vr.getTrackedDeviceClass(i)
            device = None

            # Create the right device instance based on its class
            if device_class == openvr.TrackedDeviceClass_Controller:
                device_name = "controller_" + str(len(self.object_names["Controller"]))
                device = ViveController(self.vr, i, "Controller", device_name)

            elif device_class == openvr.TrackedDeviceClass_HMD:
                device_name = "hmd_" + str(len(self.object_names["HMD"]))
                device = VRTrackedDevice(self.vr, i, "HMD", device_name)

            elif device_class == openvr.TrackedDeviceClass_GenericTracker:
                device_name = "tracker_" + str(len(self.object_names["Tracker"]))
                device = VRTrackedDevice(self.vr, i, "Tracker", device_name)

            elif device_class == openvr.TrackedDeviceClass_TrackingReference:
                device_name = "tracking_reference_" + str(len(self.object_names["Tracking Reference"]))
                device = VRTrackingReference(self.vr, i, "Tracking Reference", device_name)

            # If a device instance has been made, add it to the data structures
            if device is not None:
                self.device_serials.add(device.get_serial())
                self.object_names[device.device_class].append(device.name)
                self.devices[device.name] = device

    def wait_update_poses(self):
        if self.is_shutdown:
            return
        poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
        poses = poses_t()
        openvr.VRCompositor().waitGetPoses(poses, len(poses), None, 0)
        for device in self.devices.values():
            device.update_pose(poses)

    def get_devices(self):
        return list(self.devices.itervalues())

    def get_tracking_references(self):
        return [self.devices[name] for name in self.object_names["Tracking Reference"]]

    def get_hmds(self):
        return [self.devices[name] for name in self.object_names["HMD"]]

    def get_trackers(self):
        return [self.devices[name] for name in self.object_names["Tracker"]]

    def get_controllers(self):
        return [self.devices[name] for name in self.object_names["Controller"]]

    def get_serial(self, index):
        return self.vr.getStringTrackedDeviceProperty(index, openvr.Prop_SerialNumber_String).decode('utf-8')
