#!/usr/bin/env python3
"""Recognise a package item and its A/B/C/D letter from one camera view.

The model is the Ultralytics YOLOv8 export supplied for this project:

    input  : 1 x 3 x 640 x 640 RGB, 0..1
    output : 1 x 14 x 8400 (cx, cy, w, h + 10 class scores)
    classes: A B C D shirt toothbrush apple toilet_paper banana orange

The model is trained with objects roughly 180 px in the 640 input.  A single
full-page 640 inference misses or misclassifies large/close items, so each
frame is run through a 180/320/640 content-size pyramid and the detections are
merged with class-aware NMS.

It can run as the route runner's recognition hook:

    python3 route_run.py --route route.json \
      --recognition-command "/home/pi/RaspberryPi-CM5/xgovenv/bin/python \
        /home/pi/oumax-lidar-console/recognize_at_point.py \
        --model /home/pi/ros_ws/models/model.onnx"

The script grabs frames from the existing lidar-console MJPEG stream, runs the
model for a few frames, pairs the item with the letter above it, prints one JSON
line and writes a JSON + annotated JPEG under --out-dir.
"""
from __future__ import annotations

import argparse
import ast
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

DEFAULT_MODEL = os.environ.get("ROUTE_RECOGNITION_MODEL", "/home/pi/ros_ws/models/model.onnx")
DEFAULT_CAMERA = "http://127.0.0.1:8088/camera.mjpg"
DEFAULT_CLASSES = {
    0: "A", 1: "B", 2: "C", 3: "D",
    4: "shirt", 5: "toothbrush", 6: "apple", 7: "toilet_paper", 8: "banana", 9: "orange",
}
LETTERS = {"A", "B", "C", "D"}
RED_ITEMS = {"shirt", "toothbrush", "toilet_paper"}
BLUE_ITEMS = {"apple", "banana", "orange"}


def color_for_item(item: str | None) -> str | None:
    if item in RED_ITEMS:
        return "red"
    if item in BLUE_ITEMS:
        return "blue"
    return None


def letterbox(image: np.ndarray, size: int, content: int | None = None) -> tuple[np.ndarray, float, int, int]:
    import cv2

    content = size if content is None else int(content)
    h, w = image.shape[:2]
    scale = min(content / max(w, 1), content / max(h, 1))
    nw, nh = max(1, round(w * scale)), max(1, round(h * scale))
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x = (size - nw) // 2
    pad_y = (size - nh) // 2
    canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
    return canvas, scale, pad_x, pad_y


def class_names(session: Any) -> dict[int, str]:
    try:
        raw = session.get_modelmeta().custom_metadata_map.get("names", "")
        parsed = ast.literal_eval(raw)
        names = {int(key): str(value) for key, value in parsed.items()}
        if names:
            return names
    except (ValueError, SyntaxError, TypeError, AttributeError):
        pass
    return dict(DEFAULT_CLASSES)


def nms_classwise(boxes: np.ndarray, scores: np.ndarray, classes: np.ndarray, iou_threshold: float) -> list[int]:
    keep_all: list[int] = []
    for cls in np.unique(classes):
        idx = np.where(classes == cls)[0]
        order = idx[np.argsort(-scores[idx])]
        while order.size:
            current = int(order[0])
            keep_all.append(current)
            if order.size == 1:
                break
            rest = order[1:]
            xx1 = np.maximum(boxes[current, 0], boxes[rest, 0])
            yy1 = np.maximum(boxes[current, 1], boxes[rest, 1])
            xx2 = np.minimum(boxes[current, 2], boxes[rest, 2])
            yy2 = np.minimum(boxes[current, 3], boxes[rest, 3])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            area_current = (boxes[current, 2] - boxes[current, 0]) * (boxes[current, 3] - boxes[current, 1])
            area_rest = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
            union = area_current + area_rest - inter
            iou = np.where(union > 0, inter / union, 0.0)
            order = rest[iou <= iou_threshold]
    keep_all.sort(key=lambda index: -float(scores[index]))
    return keep_all


def decode_output(
    output: np.ndarray,
    names: dict[int, str],
    conf_threshold: float,
    iou_threshold: float,
    image_width: int,
    image_height: int,
    scale: float,
    pad_x: int,
    pad_y: int,
) -> list[dict[str, Any]]:
    arr = np.asarray(output, dtype=np.float32)
    if arr.ndim == 3:
        arr = arr[0]
    count = len(names)
    objectness = False
    if arr.shape[0] == 4 + count:
        pred = arr.T
    elif arr.shape[1] == 4 + count:
        pred = arr
    elif arr.shape[0] == 5 + count:
        pred = arr.T
        objectness = True
    elif arr.shape[1] == 5 + count:
        pred = arr
        objectness = True
    else:
        raise ValueError(f"unexpected model output shape: {arr.shape}")

    boxes_xywh = pred[:, :4]
    if objectness:
        obj = pred[:, 4]
        class_scores = pred[:, 5:] * obj[:, None]
    else:
        class_scores = pred[:, 4:]
    classes = class_scores.argmax(axis=1)
    scores = class_scores.max(axis=1)
    mask = scores >= conf_threshold
    boxes_xywh = boxes_xywh[mask]
    scores = scores[mask]
    classes = classes[mask]
    if len(scores) == 0:
        return []
    boxes = np.empty_like(boxes_xywh)
    boxes[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2
    boxes[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2
    boxes[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2
    boxes[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2
    keep = nms_classwise(boxes, scores, classes, iou_threshold)

    detections: list[dict[str, Any]] = []
    for index in keep:
        x1 = (float(boxes[index, 0]) - pad_x) / scale
        y1 = (float(boxes[index, 1]) - pad_y) / scale
        x2 = (float(boxes[index, 2]) - pad_x) / scale
        y2 = (float(boxes[index, 3]) - pad_y) / scale
        x1 = max(0.0, min(float(image_width), x1))
        x2 = max(0.0, min(float(image_width), x2))
        y1 = max(0.0, min(float(image_height), y1))
        y2 = max(0.0, min(float(image_height), y2))
        name = names.get(int(classes[index]), str(int(classes[index])))
        detections.append({
            "class": name,
            "confidence": round(float(scores[index]), 4),
            "box": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
            "cx": (x1 + x2) / 2,
            "cy": (y1 + y2) / 2,
            "w": max(1.0, x2 - x1),
            "h": max(1.0, y2 - y1),
        })
    return detections


def pair_item_letter(
    detections: list[dict[str, Any]],
    item_threshold: float = 0.30,
    letter_threshold: float = 0.30,
) -> tuple[dict[str, Any] | None, dict[str, Any] | None, float]:
    letters = [d for d in detections if d["class"] in LETTERS and d["confidence"] >= letter_threshold]
    items = [d for d in detections if (d["class"] in RED_ITEMS or d["class"] in BLUE_ITEMS) and d["confidence"] >= item_threshold]
    best_item = max(items, key=lambda d: d["confidence"], default=None)
    best_letter = max(letters, key=lambda d: d["confidence"], default=None)
    if not items or not letters:
        return best_item, best_letter, -1.0

    best: tuple[dict[str, Any], dict[str, Any], float] | None = None
    for item in items:
        for letter in letters:
            dx = abs(letter["cx"] - item["cx"]) / item["w"]
            dy = abs(letter["cy"] - item["cy"]) / item["h"]
            above = letter["cy"] < item["cy"]
            # Item and letter are printed on the same box side; prefer the
            # letter above the item and near its horizontal centre.
            score = item["confidence"] + letter["confidence"] - 0.8 * dx - (0.1 if above else 0.65)
            if dy > 2.2:
                score -= 0.3
            if best is None or score > best[2]:
                best = (item, letter, score)
    if best is None:
        return best_item, best_letter, -1.0
    return best[0], best[1], best[2]


def merge_detections(detections: list[dict[str, Any]], iou_threshold: float) -> list[dict[str, Any]]:
    if not detections:
        return []
    boxes = np.asarray([d["box"] for d in detections], dtype=np.float32)
    scores = np.asarray([d["confidence"] for d in detections], dtype=np.float32)
    classes = np.asarray([d["class"] for d in detections], dtype=object)
    # nms_classwise compares class ids; map names to stable integer ids.
    ids = {name: index for index, name in enumerate(sorted(set(classes.tolist())))}
    class_ids = np.asarray([ids[name] for name in classes], dtype=np.int32)
    keep = nms_classwise(boxes, scores, class_ids, iou_threshold)
    return [detections[index] for index in keep]


def run_frame(
    session: Any,
    image: np.ndarray,
    names: dict[int, str],
    input_name: str,
    input_size: int,
    conf: float,
    iou: float,
    item_threshold: float = 0.30,
    letter_threshold: float = 0.30,
) -> tuple[dict[str, Any], list[dict[str, Any]], None]:
    h, w = image.shape[:2]
    # The model is trained with objects around 180 px in the 640 input.
    # A 180/320/640 pyramid covers close-up box sides and more distant views.
    variants = [
        ((0, 0, w, h), max(160, round(input_size * 0.28))),
        ((0, 0, w, h), max(240, input_size // 2)),
        ((0, 0, w, h), input_size),
    ]
    all_detections: list[dict[str, Any]] = []
    for (x0, y0, x1, y1), content in variants:
        crop = image[y0:y1, x0:x1]
        if crop.size == 0:
            continue
        padded, scale, pad_x, pad_y = letterbox(crop, input_size, content)
        rgb = padded[:, :, ::-1].astype(np.float32) / 255.0
        tensor = np.transpose(rgb, (2, 0, 1))[None, ...]
        output = session.run(None, {input_name: tensor})[0]
        detections = decode_output(
            output, names, conf, iou, crop.shape[1], crop.shape[0], scale, pad_x, pad_y,
        )
        for det in detections:
            bx1, by1, bx2, by2 = det["box"]
            det["box"] = [round(bx1 + x0, 1), round(by1 + y0, 1), round(bx2 + x0, 1), round(by2 + y0, 1)]
            det["cx"] += x0
            det["cy"] += y0
        all_detections.extend(detections)
    detections = merge_detections(all_detections, iou)
    item, letter, pair_score = pair_item_letter(detections, item_threshold, letter_threshold)
    result = {
        "item": item["class"] if item else None,
        "letter": letter["class"] if letter else None,
        "color": color_for_item(item["class"]) if item else None,
        "item_confidence": item["confidence"] if item else None,
        "letter_confidence": letter["confidence"] if letter else None,
        "pair_score": round(pair_score, 4) if pair_score >= 0 else None,
        "variants": len(variants),
        "detections": detections,
    }
    return result, detections, None


def annotate(image: np.ndarray, detections: list[dict[str, Any]], summary: dict[str, Any]) -> np.ndarray:
    import cv2

    canvas = image.copy()
    for det in detections:
        x1, y1, x2, y2 = [int(round(v)) for v in det["box"]]
        is_letter = det["class"] in LETTERS
        color = (20, 150, 90) if is_letter else (20, 100, 180)
        if det["class"] == summary.get("letter") or det["class"] == summary.get("item"):
            color = (0, 80, 230)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 3)
        label = f"{det['class']} {det['confidence']:.2f}"
        cv2.putText(canvas, label, (x1, max(18, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
    text = f"letter={summary.get('letter')} item={summary.get('item')} color={summary.get('color')}"
    cv2.putText(canvas, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 80, 230), 2, cv2.LINE_AA)
    return canvas


def choose_best(results: list[dict[str, Any]]) -> dict[str, Any]:
    def score(result: dict[str, Any]) -> float:
        item_conf = result.get("item_confidence") or 0.0
        letter_conf = result.get("letter_confidence") or 0.0
        pair = result.get("pair_score") or 0.0
        both = 1.0 if item_conf and letter_conf else 0.0
        return both * 10.0 + item_conf + letter_conf + pair

    return max(results, key=score)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Recognise package item + A/B/C/D letter with model.onnx.")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="ONNX model path")
    parser.add_argument("--image", default=None, help="run on one image instead of the camera")
    parser.add_argument("--camera-url", default=DEFAULT_CAMERA, help="MJPEG stream")
    parser.add_argument("--frames", type=int, default=2, help="camera frames to aggregate")
    parser.add_argument("--interval", type=float, default=0.35, help="seconds between camera frames")
    parser.add_argument("--conf", type=float, default=0.20, help="raw detection confidence threshold")
    parser.add_argument("--item-conf", type=float, default=0.30, help="minimum item confidence for a result")
    parser.add_argument("--letter-conf", type=float, default=0.30, help="minimum letter confidence for a result")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold")
    parser.add_argument("--out-dir", default="recognition", help="result directory")
    parser.add_argument("--point", default=os.environ.get("ROUTE_POINT"), help="route waypoint number")
    parser.add_argument("--require", choices=("any", "both", "letter", "item"), default="both",
                        help="what counts as success (exit code 0)")
    parser.add_argument("--no-save", action="store_true", help="do not write result files")
    args = parser.parse_args(argv)

    try:
        import cv2
        import onnxruntime as ort
    except Exception as exc:
        print(f"缺少识别依赖 (opencv/onnxruntime): {exc}", file=sys.stderr)
        return 2
    model_path = Path(args.model)
    if not model_path.is_file():
        print(f"模型不存在: {model_path}", file=sys.stderr)
        return 2
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_info = session.get_inputs()[0]
    input_name = input_info.name
    shape = input_info.shape
    input_size = int(shape[-1]) if isinstance(shape[-1], int) else 640
    names = class_names(session)

    frames: list[tuple[np.ndarray, str]] = []
    camera = None
    if args.image:
        image = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if image is None:
            print(f"无法读取图片: {args.image}", file=sys.stderr)
            return 2
        frames.append((image, "image"))
    else:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from capture_photos import CameraFrames

        camera = CameraFrames(args.camera_url)
        camera.thread.start()
        earliest = 0.0
        try:
            for index in range(max(1, args.frames)):
                if index:
                    time.sleep(max(0.0, args.interval))
                stamp, _, jpeg = camera.frame_after(earliest)
                earliest = stamp + 1e-9
                image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError("camera frame decode failed")
                frames.append((image, f"frame-{index}"))
        except Exception as exc:
            print(f"摄像头取帧失败: {exc}", file=sys.stderr)
            if camera:
                camera.stopped.set()
            return 2

    results: list[dict[str, Any]] = []
    chosen_image = None
    try:
        for image, source in frames:
            result, detections, _ = run_frame(
                session, image, names, input_name, input_size, args.conf, args.iou,
                args.item_conf, args.letter_conf,
            )
            result["source"] = source
            results.append(result)
            if chosen_image is None:
                chosen_image = image
        best = choose_best(results)
        # Re-annotate the frame that produced the chosen result.
        best_index = results.index(best)
        chosen_image = frames[best_index][0]
        annotated = annotate(chosen_image, best["detections"], best)
    finally:
        if camera:
            camera.stopped.set()

    point = None
    if args.point is not None:
        try:
            point = int(args.point)
        except (TypeError, ValueError):
            point = args.point
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    out_dir = Path(args.out_dir).expanduser()
    output = {
        "ok": bool(best.get("item") and best.get("letter")),
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "point": point,
        "model": str(model_path),
        "classes": {str(key): value for key, value in names.items()},
        "frames": len(results),
        "variants": best.get("variants", 0),
        "letter": best.get("letter"),
        "item": best.get("item"),
        "color": best.get("color"),
        "letter_confidence": best.get("letter_confidence"),
        "item_confidence": best.get("item_confidence"),
        "pair_score": best.get("pair_score"),
        "detections": best.get("detections", []),
        "all_frames": results,
        "image": None,
    }
    if not args.no_save:
        out_dir.mkdir(parents=True, exist_ok=True)
        prefix = f"point-{point}" if point is not None else "manual"
        image_path = out_dir / f"{prefix}-{timestamp}.jpg"
        result_path = out_dir / f"{prefix}-{timestamp}.json"
        cv2.imwrite(str(image_path), annotated)
        output["image"] = str(image_path)
        result_path.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"result: {result_path}", file=sys.stderr)

    print(json.dumps(output, ensure_ascii=False))
    print(
        f"point={point} letter={output['letter']} item={output['item']} "
        f"color={output['color']} frames={len(results)}",
        file=sys.stderr,
    )
    required = args.require
    if required == "any":
        return 0 if (output["letter"] or output["item"]) else 2
    if required == "letter":
        return 0 if output["letter"] else 2
    if required == "item":
        return 0 if output["item"] else 2
    return 0 if (output["letter"] and output["item"]) else 2


if __name__ == "__main__":
    raise SystemExit(main())
