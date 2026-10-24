#!/usr/bin/env python3
"""On-demand multimodal Ollama fallback; never runs in the frame loop."""

import base64
import json
import socket
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from robis_interfaces.srv import DetectObject

from .detection_utils import make_detection


def parse_vlm_response(content, width, height, header):
    """Parse the small JSON contract returned by the configured VLM."""
    if isinstance(content, str):
        content = content.strip()
        if content.startswith("```json"):
            content = content[len("```json"):]
        if content.endswith("```"):
            content = content[:-len("```")]
        content = content.strip()
        content = json.loads(content)
    if not content.get("found", False):
        return None
    box = content.get("bbox", [])
    if len(box) != 4:
        raise ValueError("VLM response must contain bbox=[x_min,y_min,x_max,y_max]")
    if max(box) <= 1.0:
        box = [box[0] * width, box[1] * height, box[2] * width, box[3] * height]
    return make_detection(
        header, content.get("label", "object"), float(content.get("confidence", 0.5)), *box
    )


class OllamaDetector(Node):
    def __init__(self):
        super().__init__("ollama_detector")
        self.declare_parameter("enabled", False)
        self.declare_parameter("url", "http://127.0.0.1:11434")
        self.declare_parameter("model", "")
        self.declare_parameter("timeout_sec", 3.0)
        self.declare_parameter("retries", 1)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("service", "/semantic_perception/detect_object_vlm")
        self.declare_parameter(
            "detections_topic", "/semantic_perception/vlm_detections"
        )
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_header = None
        self.detections_publisher = self.create_publisher(
            Detection2DArray, self.get_parameter("detections_topic").value, 10
        )
        self.create_subscription(Image, self.get_parameter("image_topic").value, self._image_callback, 1)
        self.create_service(DetectObject, self.get_parameter("service").value, self._detect)

    def _image_callback(self, message):
        self.latest_frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.latest_header = message.header

    def _detect(self, request, response):
        if not bool(self.get_parameter("enabled").value):
            response.message = "Fallback Ollama desabilitado"
            return response
        model = str(self.get_parameter("model").value)
        if not model:
            response.message = "Parâmetro ollama.model não configurado"
            return response
        if self.latest_frame is None:
            response.message = "Nenhuma imagem recebida"
            return response
        ok, encoded = cv2.imencode(".jpg", self.latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            response.message = "Falha ao codificar a imagem"
            return response
        prompt = (
            f"Encontre '{request.query}' na imagem. Responda somente JSON: "
            '{"found":bool,"label":str,"confidence":0..1,"bbox":[x_min,y_min,x_max,y_max]}. '
            "Use coordenadas normalizadas entre 0 e 1. Se ausente, found=false."
        )
        payload = {
            "model": model, "stream": False,
            "messages": [{"role": "user", "content": prompt,
                          "images": [base64.b64encode(encoded.tobytes()).decode("ascii")]}],
            "format": "json",
        }
        url = str(self.get_parameter("url").value).rstrip("/") + "/api/chat"
        attempts = max(1, int(self.get_parameter("retries").value) + 1)
        for attempt in range(attempts):
            try:
                http_request = Request(url, data=json.dumps(payload).encode("utf-8"),
                                       headers={"Content-Type": "application/json"}, method="POST")
                with urlopen(http_request, timeout=float(self.get_parameter("timeout_sec").value)) as stream:
                    body = json.loads(stream.read().decode("utf-8"))
                detection = parse_vlm_response(
                    body.get("message", {}).get("content", "{}"),
                    self.latest_frame.shape[1], self.latest_frame.shape[0], self.latest_header
                )
                if detection is None:
                    response.message = "Objeto não confirmado pelo VLM"
                    return response
                detection.results[0].hypothesis.class_id = " ".join(
                    request.query.strip().lower().split()
                )
                response.found, response.detection = True, detection
                response.message = "Objeto confirmado pelo VLM"
                detections = Detection2DArray()
                detections.header = self.latest_header
                detections.detections.append(detection)
                self.detections_publisher.publish(detections)
                return response
            except (HTTPError, URLError, socket.timeout, TimeoutError, ValueError, json.JSONDecodeError) as error:
                response.message = f"Ollama indisponível: {error}"
                if attempt + 1 == attempts:
                    self.get_logger().warning(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OllamaDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
