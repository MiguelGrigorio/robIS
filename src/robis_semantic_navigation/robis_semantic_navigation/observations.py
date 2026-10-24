"""Fresh, per-label observation cache for asynchronous detector backends."""

import time


class ObservationCache:
    """Keep the newest non-empty detector frame for every semantic label."""

    def __init__(self):
        self._sequence = 0
        self._entries = {}
        self.last_message_time = 0.0

    def update(self, detections_by_label, received_at=None):
        """Record a detector frame without erasing other still-fresh labels."""
        now = time.monotonic() if received_at is None else float(received_at)
        self._sequence += 1
        self.last_message_time = now
        for label, detections in detections_by_label.items():
            if detections:
                self._entries[label] = (self._sequence, now, list(detections))
        return self._sequence

    def snapshot(self, label, max_age, now=None):
        """Return ``(sequence, detections)`` when the label is still fresh."""
        current = time.monotonic() if now is None else float(now)
        entry = self._entries.get(label)
        if entry is None:
            return None
        sequence, received_at, detections = entry
        if current - received_at > float(max_age):
            return None
        return sequence, list(detections)

    def is_alive(self, max_age, now=None):
        current = time.monotonic() if now is None else float(now)
        return (
            self.last_message_time > 0.0
            and current - self.last_message_time <= float(max_age)
        )
