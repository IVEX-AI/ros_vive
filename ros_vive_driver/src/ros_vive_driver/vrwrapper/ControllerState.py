class ControllerState(object):

    def __init__(self, trigger, trackpad_x, trackpad_y, menu_button, trackpad_pressed, trackpad_touched, grip_button):
        self.trigger = trigger
        self.trackpad_x = trackpad_x
        self.trackpad_y = trackpad_y
        self.menu_button = menu_button
        self.trackpad_pressed = trackpad_pressed
        self.trackpad_touched = trackpad_touched
        self.grip_button = grip_button