import openvr

from .VRTrackedDevice import VRTrackedDevice


class VRTrackingReference(VRTrackedDevice):
    def get_mode(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModeLabel_String).decode('utf-8').upper()
