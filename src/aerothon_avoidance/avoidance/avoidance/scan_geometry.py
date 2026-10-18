"""Lidar rays registered using sensor extrinsics and capture-time attitude."""
import math
import numpy as np


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-6:
        raise ValueError('invalid sensor or aircraft orientation')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def register_rays(bearings, ranges, mount_translation, mount_quaternion,
                  capture_xy, capture_quaternion, current_xy, current_yaw):
    """Return ray origin, angles and projected depths in current body XY.

    Keep the translated sensor origin for free-space checks. Recasting rays
    from the aircraft centre would invent visibility behind nearby objects.
    """
    aircraft = quaternion_rotation(capture_quaternion)
    mount = quaternion_rotation(mount_quaternion)
    if not np.isfinite([*capture_xy,*current_xy,current_yaw]).all():
        raise ValueError('invalid capture-time aircraft pose')
    c, s = math.cos(current_yaw), math.sin(current_yaw)
    body = np.array([[c, s], [-s, c]])
    translation = np.asarray(mount_translation, dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError('invalid lidar mounting translation')
    origin = body @ (np.asarray(capture_xy)-np.asarray(current_xy) +
                     (aircraft @ translation)[:2])
    rays = np.column_stack((np.cos(bearings)*ranges,
                            np.sin(bearings)*ranges, np.zeros(len(ranges))))
    projected = (rays @ (aircraft @ mount).T)[:, :2] @ body.T
    return origin, np.arctan2(projected[:, 1], projected[:, 0]), np.linalg.norm(projected, axis=1)
