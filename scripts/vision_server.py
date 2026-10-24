#!/usr/bin/env python3
"""Image inference HTTP service for an Orin used as a robot's vision server.

YOLO/Florence return boxes; SmolVLM returns a presence answer, not fabricated
coordinates. Image identifiers and capture metadata are echoed unchanged.
"""
import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
import threading
import time

from PIL import Image
import torch


class Models:
    def __init__(self, names):
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is required; CPU fallback is disabled')
        torch.set_num_threads(2)
        self.items = {}
        if 'yolo' in names:
            from ultralytics import YOLO
            self.items['yolo'] = YOLO('/opt/robis-models/yolo26n.pt')
        if 'smol' in names:
            from transformers import AutoModelForImageTextToText, AutoProcessor
            path = '/opt/robis-models/smolvlm-256m'
            self.items['smol'] = (
                AutoProcessor.from_pretrained(path, local_files_only=True),
                AutoModelForImageTextToText.from_pretrained(path, local_files_only=True,
                    torch_dtype=torch.float16, attn_implementation='eager').cuda().eval())
        if 'florence' in names:
            from transformers import AutoProcessor, Florence2ForConditionalGeneration
            path = '/opt/robis-models/florence2-base-ft'
            self.items['florence'] = (
                AutoProcessor.from_pretrained(path, local_files_only=True),
                Florence2ForConditionalGeneration.from_pretrained(path, local_files_only=True,
                    torch_dtype=torch.float16, attn_implementation='eager').cuda().eval())

    @torch.inference_mode()
    def infer(self, backend, image, query):
        if backend == 'yolo':
            result = self.items[backend].predict(image, imgsz=640, device='0',
                                                conf=0.25, verbose=False)[0]
            return {'task': 'detection', 'detections': [
                {'label': result.names[int(box.cls[0])], 'score': float(box.conf[0]),
                 'bbox': box.xyxy[0].cpu().tolist()} for box in result.boxes]}
        processor, model = self.items[backend]
        if backend == 'smol':
            prompt = f'Is there at least one {query} visible in this image? Answer only yes or no.'
            message = [{'role': 'user', 'content': [
                {'type': 'image'}, {'type': 'text', 'text': prompt}]}]
            text = processor.apply_chat_template(message, add_generation_prompt=True, tokenize=False)
            resized = image.copy()
            resized.thumbnail((512, 512))
            inputs = processor(text=text, images=[resized], return_tensors='pt').to('cuda', torch.float16)
            output = model.generate(**inputs, max_new_tokens=8, do_sample=False)
            answer = processor.batch_decode(output[:, inputs['input_ids'].shape[1]:],
                                             skip_special_tokens=True)[0].strip()
            token = answer.lower().strip('.! \n')
            return {'task': 'presence', 'answer': answer,
                    'present': True if token == 'yes' else False if token == 'no' else None,
                    'detections': []}
        inputs = processor(text='<OD>', images=image, return_tensors='pt').to('cuda', torch.float16)
        output = model.generate(**inputs, max_new_tokens=512, num_beams=1, do_sample=False)
        text = processor.batch_decode(output, skip_special_tokens=False)[0]
        result = processor.post_process_generation(text, task='<OD>', image_size=image.size)['<OD>']
        return {'task': 'detection', 'detections': [
            {'label': label, 'score': None, 'bbox': box}
            for label, box in zip(result['labels'], result['bboxes'])]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--models', default='yolo,smol,florence')
    args = parser.parse_args()
    names = args.models.split(',')
    if not set(names) <= {'yolo', 'smol', 'florence'}:
        raise SystemExit('Unknown model backend')
    models = Models(names)
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def send_json(self, status, result):
            encoded = json.dumps(result, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

        def do_GET(self):
            self.send_json(200 if self.path == '/health' else 404,
                           {'models': names, 'cuda': True, 'gpu': torch.cuda.get_device_name(0)})

        def do_POST(self):
            if self.path != '/infer':
                self.send_json(404, {'error': 'Unknown endpoint'})
                return
            self.connection.settimeout(30)
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4*1024*1024:
                    raise ValueError('Image request must be between 1 byte and 4 MiB')
                request = json.loads(self.rfile.read(length))
                backend = request['backend']
                if backend not in models.items:
                    raise ValueError('Backend not loaded')
                query = request.get('query', 'chair')
                if not isinstance(query, str) or not 0 < len(query) <= 128:
                    raise ValueError('Query must be a short object name')
                encoded = base64.b64decode(request['image'], validate=True)
                with Image.open(BytesIO(encoded)) as source:
                    if source.width*source.height > 4096*4096:
                        raise ValueError('Image dimensions exceed the experiment limit')
                    image = source.convert('RGB')
            except (ValueError, KeyError, OSError) as error:
                self.send_json(400, {'error': str(error)})
                return
            if not lock.acquire(blocking=False):
                self.send_json(503, {'error': 'Inference busy; keep only the most recent frame'})
                return
            try:
                torch.cuda.synchronize()
                started = time.perf_counter()
                result = models.infer(backend, image, query)
                torch.cuda.synchronize()
                result.update(backend=backend, width=image.width, height=image.height,
                    request_id=request.get('request_id'), frame_id=request.get('frame_id'),
                    stamp=request.get('stamp'), coordinates='xyxy_pixels',
                    inference_ms=(time.perf_counter()-started)*1000)
                self.send_json(200, result)
            except Exception as error:
                self.send_json(500, {'error': str(error), 'backend': backend})
            finally:
                lock.release()

    print(json.dumps({'ready': True, 'models': names, 'port': args.port}), flush=True)
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
