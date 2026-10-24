"""Project the known robot volume into calibrated images using measured TF."""
from itertools import product

import cv2
import numpy as np


def transform_matrix(transform):
    q = transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid robot TF rotation')
    x, y, z, w = quaternion / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    if not np.isfinite(matrix).all():
        raise ValueError('non-finite robot TF')
    return matrix


def robot_polygon(camera, map_from_robot, map_rotation, map_translation, dimensions):
    """Return a convex image polygon; all geometry is in metres, no object truth."""
    length, width, height = dimensions
    if not all(np.isfinite(dimensions)) or min(dimensions) <= 0:
        raise ValueError('robot dimensions must be finite and positive')
    points = np.array(list(product([-length / 2, length / 2], [-width / 2, width / 2], [0., height])))
    points = points @ map_from_robot[:3, :3].T + map_from_robot[:3, 3]
    points[:, :2] = (points[:, :2] - map_translation) @ map_rotation
    camera_from_world = np.linalg.inv(camera.world_from_camera)
    optical = points @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
    if np.any(optical[:, 2] <= 0):
        return None
    rotation, _ = cv2.Rodrigues(camera_from_world[:3, :3])
    pixels, _ = cv2.projectPoints(points, rotation, camera_from_world[:3, 3],
                                  camera.matrix, camera.distortion)
    return cv2.convexHull(pixels.astype(np.float32).reshape(-1, 2))


def overlaps_robot(detection, polygon, margin_px=4.):
    if polygon is None:
        return False
    x1, y1, x2, y2 = detection['bbox']
    centre = ((x1 + x2) / 2, (y1 + y2) / 2)
    if cv2.pointPolygonTest(polygon, centre, True) >= -margin_px:
        return True
    rectangle = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], np.float32)
    area, _ = cv2.intersectConvexConvex(polygon, rectangle)
    return area / max((x2 - x1) * (y2 - y1), 1.) >= 0.5
