"""Versioned and crash-resistant YAML semantic landmark store."""

from copy import deepcopy
from datetime import datetime, timezone
from functools import wraps
import math
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
import uuid

import yaml


SCHEMA_VERSION = 1


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def synchronized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return call


class SemanticStore:
    """Manage one atomic YAML document containing multiple environments."""

    def __init__(self, filename):
        self._lock = threading.RLock()
        self.path = Path(filename)
        self.backup_path = self.path.with_suffix(self.path.suffix + ".bak")
        self.data = self._load()

    @staticmethod
    def empty():
        return {"schema_version": SCHEMA_VERSION, "environments": {}}

    def _load_path(self, path):
        with path.open("r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream) or self.empty()
        if not isinstance(value, dict) or "environments" not in value:
            raise ValueError("invalid semantic map schema")
        value.setdefault("schema_version", SCHEMA_VERSION)
        return value

    def _load(self):
        if not self.path.exists():
            return self.empty()
        try:
            return self._load_path(self.path)
        except (OSError, ValueError, yaml.YAMLError):
            if self.backup_path.exists():
                return self._load_path(self.backup_path)
            raise

    @synchronized
    def environment(self, environment_id, map_checksum=""):
        environments = self.data.setdefault("environments", {})
        environment = environments.setdefault(environment_id, {
            "frame_id": "map", "map_checksum": map_checksum, "objects": {}
        })
        existing = environment.get("map_checksum", "")
        if existing and map_checksum and existing != map_checksum:
            raise ValueError("semantic map checksum does not match navigation map")
        if map_checksum and not existing:
            environment["map_checksum"] = map_checksum
        return environment

    @synchronized
    def instances(self, environment_id, label, valid_only=True, map_checksum="", ros_now_ns=None):
        environment = self.data.get("environments", {}).get(environment_id, {})
        existing = environment.get("map_checksum", "")
        if existing and map_checksum and existing != map_checksum:
            raise ValueError("semantic map checksum does not match navigation map")
        entries = deepcopy(environment.get("objects", {}).get(label, []))
        if valid_only:
            entries = [entry for entry in entries if entry.get("valid", True)
                       and float(entry.get("expires_at_unix", float("inf"))) > time.time()
                       and (ros_now_ns is None or entry.get('external_ros_deadline_ns', float('inf')) > ros_now_ns)]
        return entries

    @synchronized
    def select(self, environment_id, label, policy="nearest", robot_xy=(0.0, 0.0),
               map_checksum="", ros_now_ns=None):
        entries = self.instances(
            environment_id, label, map_checksum=map_checksum, ros_now_ns=ros_now_ns
        )
        if not entries:
            return None
        if policy == "highest_confidence":
            return max(entries, key=lambda item: float(item.get("confidence", 0.0)))
        if policy == "newest":
            return max(entries, key=lambda item: item.get("last_confirmed", item.get("timestamp", "")))
        if policy != "nearest":
            raise ValueError(f"unsupported instance policy: {policy}")
        rx, ry = robot_xy
        return min(entries, key=lambda item: math.hypot(item["pose"]["x"] - rx, item["pose"]["y"] - ry))

    @synchronized
    def upsert(self, environment_id, label, pose, confidence, merge_distance=0.5,
               map_checksum="", observation_ros_stamp_ns=None, instance_id=None):
        environment = self.environment(environment_id, map_checksum)
        entries = environment.setdefault("objects", {}).setdefault(label, [])
        closest = None
        for entry in entries:
            distance = math.hypot(entry["pose"]["x"] - pose["x"], entry["pose"]["y"] - pose["y"])
            if instance_id is not None:
                if entry['id'] == instance_id:
                    closest = (distance, entry)
                    break
                continue
            if distance <= merge_distance and (closest is None or distance < closest[0]):
                closest = (distance, entry)
        if instance_id is not None and closest is None:
            raise ValueError('confirmed instance no longer exists')
        now = utc_now()
        if closest:
            entry = closest[1]
            count = int(entry.get("observations", 1))
            # A moving CCTV target is a current measurement, not a static
            # landmark to average with its previous location.
            weight = 0 if entry.get('external_id') else min(count, 9)
            entry["pose"] = {
                "x": (entry["pose"]["x"] * weight + pose["x"]) / (weight + 1),
                "y": (entry["pose"]["y"] * weight + pose["y"]) / (weight + 1),
                "yaw": pose.get("yaw", entry["pose"].get("yaw", 0.0)),
            }
            entry["confidence"] = max(float(confidence), float(entry.get("confidence", 0.0)))
            entry["observations"] = count + 1
            entry["last_confirmed"] = now
            entry["valid"] = True
            entry.pop('onboard_rejected', None)
            if 'expires_at_unix' in entry:
                # A new onboard confirmation is a new measurement, unlike a
                # repeated external snapshot. Keep this landmark short lived.
                entry['expires_at_unix'] = time.time() + entry['external_validity_sec']
                entry['onboard_confirmations'] = entry.get('onboard_confirmations', 0) + 1
                entry['last_onboard_confirmed'] = now
                if observation_ros_stamp_ns is not None:
                    entry['external_ros_deadline_ns'] = (
                        observation_ros_stamp_ns + round(entry['external_validity_sec'] * 1e9))
        else:
            entry = {
                "id": str(uuid.uuid4()), "pose": dict(pose), "timestamp": now,
                "last_confirmed": now, "confidence": float(confidence),
                "observations": 1, "valid": True,
            }
            entries.append(entry)
        self.save()
        return deepcopy(entry)

    @synchronized
    def upsert_external(self, environment_id, label, external_id, pose, confidence,
                        stamp_ns, expires_at, map_checksum, revision=0, validity_sec=None):
        """Replace a live object's pose; duplicate frames never extend its expiry."""
        if (not external_id or not label or not map_checksum or stamp_ns <= 0
                or not all(math.isfinite(float(v)) for v in
                           [pose['x'], pose['y'], confidence, expires_at])
                or not 0 <= confidence <= 1 or expires_at <= time.time()):
            raise ValueError("invalid or expired external observation")
        environment = self.environment(environment_id, map_checksum)
        entries = environment.setdefault("objects", {}).setdefault(label, [])
        entry = next((item for item in entries if item.get('external_id') == external_id), None)
        if entry is not None and entry.get('onboard_rejected', False):
            return None
        if entry is not None:
            last_stamp = entry.get('external_stamp_ns', 0)
            if stamp_ns < last_stamp:
                return None
            if stamp_ns == last_stamp:
                if (revision <= entry.get('external_revision', 0)
                        or entry['expires_at_unix'] <= time.time()):
                    return None
                # Another view can refine the same captured instant, but cannot
                # renew freshness just because its inference arrived later.
                expires_at = entry['expires_at_unix']
        now = utc_now()
        if entry is None:
            entry = {'id': str(uuid.uuid4()), 'external_id': external_id,
                     'timestamp': now, 'observations': 0}
            entries.append(entry)
        entry.update(pose=dict(pose), confidence=float(confidence), valid=True,
                     source='fixed_cameras', external_stamp_ns=int(stamp_ns),
                     external_revision=int(revision),
                     expires_at_unix=float(expires_at), last_confirmed=now,
                     external_validity_sec=max(0.01, float(expires_at) - time.time()),
                     observations=entry['observations'] + 1)
        if validity_sec is not None:
            entry['external_ros_deadline_ns'] = stamp_ns + round(validity_sec * 1e9)
        self.save()
        return deepcopy(entry)

    @synchronized
    def mark_stale(self, environment_id, label, instance_id, decay=0.5):
        environment = self.environment(environment_id)
        for entry in environment.get("objects", {}).get(label, []):
            if entry.get("id") == instance_id:
                entry["confidence"] = float(entry.get("confidence", 0.0)) * decay
                # This path is called only after the robot reached the saved
                # pose and an active visual scan failed. Keep the record for
                # auditability, but never select it as a navigation target.
                entry["valid"] = False
                if entry.get('external_id'):
                    entry['onboard_rejected'] = True
                entry["stale_since"] = utc_now()
                self.save()
                return

    @synchronized
    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            shutil.copy2(self.path, self.backup_path)
        descriptor, temporary = tempfile.mkstemp(prefix=self.path.name + ".", suffix=".tmp", dir=self.path.parent)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(self.data, stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory_fd = os.open(self.path.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
