"""Helpers shared by semantic detector nodes."""

from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose


def make_detection(header, class_id, score, x_min, y_min, x_max, y_max):
    detection = Detection2D()
    detection.header = header
    center_x = (float(x_min) + float(x_max)) / 2.0
    center_y = (float(y_min) + float(y_max)) / 2.0
    if hasattr(detection.bbox.center, "position"):
        detection.bbox.center.position.x = center_x
        detection.bbox.center.position.y = center_y
    else:
        detection.bbox.center.x = center_x
        detection.bbox.center.y = center_y
    detection.bbox.size_x = float(x_max) - float(x_min)
    detection.bbox.size_y = float(y_max) - float(y_min)
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = str(class_id)
    result.hypothesis.score = float(score)
    detection.results.append(result)
    return detection
