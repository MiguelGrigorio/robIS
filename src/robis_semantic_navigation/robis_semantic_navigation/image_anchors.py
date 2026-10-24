"""Choose image anchors on broad segmented surfaces, avoiding thin furniture legs."""
import cv2
import numpy as np


def mask_incenter(mask, bbox):
    """Return an interior point in the thickest region of the instance mask."""
    binary = (np.asarray(mask) > .5).astype(np.uint8)
    height, width = binary.shape
    x1, y1, x2, y2 = bbox
    left, top = max(0, int(np.floor(x1))), max(0, int(np.floor(y1)))
    right, bottom = min(width, int(np.ceil(x2))), min(height, int(np.ceil(y2)))
    if right <= left or bottom <= top:
        return None
    cropped = binary[top:bottom, left:right]
    distances = cv2.distanceTransform(np.pad(cropped, 1), cv2.DIST_L2, 5)[1:-1, 1:-1]
    maximum = float(distances.max())
    if maximum < 2.:
        return None
    pixels = np.argwhere(distances >= .9 * maximum)
    centroid = pixels.mean(axis=0)
    y, x = pixels[np.argmin(np.sum((pixels - centroid)**2, axis=1))]
    anchor = [float(x + left), float(y + top)]
    return anchor if x1 <= anchor[0] <= x2 and y1 <= anchor[1] <= y2 else None


def yolo_detections(result, planes, width, height):
    """Use configured anchors identically for a live stream and image replay."""
    masks = None
    if (any(plane.get('anchor_method') == 'mask_incenter' for plane in planes.values())
            and result.masks is not None):
        masks = result.masks.data.detach().cpu().numpy()
    detections = []
    for index, box in enumerate(result.boxes):
        label = result.names[int(box.cls[0])]
        detection = {'label': label, 'score': float(box.conf[0]), 'bbox': box.xyxy[0].cpu().tolist()}
        if planes.get(label, {}).get('anchor_method') == 'mask_incenter':
            if masks is None or index >= len(masks):
                continue
            mask = cv2.resize(masks[index], (width, height), interpolation=cv2.INTER_NEAREST)
            anchor = mask_incenter(mask, detection['bbox'])
            if anchor is None:
                continue
            detection['anchor'] = anchor
            detection['anchor_method'] = 'mask_incenter'
        detections.append(detection)
    return detections
