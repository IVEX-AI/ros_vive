import time

from .VRTrackedDevice import VRTrackedDevice
from .AsyncExclusiveActionExecutor import AsyncExclusiveActionExecutor
from .ControllerState import ControllerState


class ViveController(VRTrackedDevice):
    VIBRATION_LOOP_RATE = 250

    def __init__(self, vr_obj, index, device_class, name):
        super(ViveController, self).__init__(vr_obj, index, device_class, name)
        self.vibration_executor = AsyncExclusiveActionExecutor()

    def shutdown(self):
        super(ViveController, self).shutdown()
        self.vibration_executor.shutdown()

    def get_controller_state(self):
        result, p_controller_state = self.vr.getControllerState(self.index)
        # from: https://gist.github.com/awesomebytes/75daab3adb62b331f21ecf3a03b3ab46
        # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState

        # on trigger .y is always 0.0 says the docs
        trigger = p_controller_state.rAxis[1].x
        # 0.0 on trigger is fully released
        # -1.0 to 1.0 on joystick and trackpads
        trackpad_x = p_controller_state.rAxis[0].x
        trackpad_y = p_controller_state.rAxis[0].y

        # Second bit marks menu button
        menu_button = bool(p_controller_state.ulButtonPressed >> 1 & 1)

        # 32 bit marks trackpad
        trackpad_pressed = bool(p_controller_state.ulButtonPressed >> 32 & 1)
        trackpad_touched = bool(p_controller_state.ulButtonTouched >> 32 & 1)

        # third bit marks grip button
        grip_button = bool(p_controller_state.ulButtonPressed >> 2 & 1)

        # System button can't be read, if you press it
        # the controllers stop reporting
        return ControllerState(trigger, trackpad_x, trackpad_y, menu_button, trackpad_pressed, trackpad_touched,
                               grip_button)

    def start_continuous_vibration(self, strength):
        self.vibration_executor.queue_action(ViveController._vibration_pulse_callback,
                                             [self.vr, self.index, strength, -1])

    def start_vibration_pulse(self, strength, duration):
        self.vibration_executor.queue_action(ViveController._vibration_pulse_callback,
                                             [self.vr, self.index, strength, duration])

    def start_multiple_vibration_pulses(self, strength, pulse_duration, pulse_count, pulse_period):
        self.vibration_executor.queue_action(ViveController._multiple_vibration_pulses_callback,
                                             [self.vr, self.index, strength, pulse_duration, pulse_count, pulse_period])

    @staticmethod
    def _convert_vibration_strength(strength):
        if strength >= 1.0:
            return 3999
        elif strength <= 0:
            return 0
        else:
            return int(strength * 3999)

    @staticmethod
    def _vibration_pulse_callback(vr, index, strength, duration, stop_event):
        start = time.time()
        while duration < 0 or start + duration > time.time():
            if stop_event.is_set():
                break
            vr.triggerHapticPulse(index, 0, ViveController._convert_vibration_strength(strength))
            stop_event.wait(1.0 / ViveController.VIBRATION_LOOP_RATE)

    @staticmethod
    def _multiple_vibration_pulses_callback(vr, index, strength, pulse_duration, count, pulse_period, stop_event):
        for i in range(count):
            if stop_event.is_set():
                break
            ViveController._vibration_pulse_callback(vr, index, strength, pulse_duration, stop_event)
            stop_event.wait(pulse_period)
