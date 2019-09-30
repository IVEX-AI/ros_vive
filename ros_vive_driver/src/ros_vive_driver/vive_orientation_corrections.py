import math

from tf import transformations as t

DEVICE_ORIENTATION_CORRECTION = {
    "Tracker": t.euler_matrix(math.pi, 0, math.pi / 2)  # Z points downwards and X points to the right
}


def correct_orientation(device, orientation):
    if device.device_class in DEVICE_ORIENTATION_CORRECTION:
        start_orient_mtx = t.quaternion_matrix(orientation)
        correction = DEVICE_ORIENTATION_CORRECTION[device.device_class]
        result_mtx = t.concatenate_matrices(start_orient_mtx, correction)
        result_list = t.quaternion_from_matrix(result_mtx)
        return tuple(result_list)
    else:
        return orientation
