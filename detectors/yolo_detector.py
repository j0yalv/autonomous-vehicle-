import cv2
import numpy as np
import tempfile
import os

COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus",
    "train", "truck", "boat", "traffic light", "fire hydrant",
    "stop sign", "parking meter", "bench", "bird", "cat", "dog"
]


class YOLODetector:

    TARGET_CLASSES = {
        "person",
        "bicycle",
        "car",
        "motorcycle",
        "bus",
        "truck",
    }

    def __init__(self, model_name="yolov8n.pt", confidence=0.4):

        self.model_name = model_name
        self.confidence = confidence
        self.model = None
        self.enabled = False

        # Run YOLO every N frames
        self.frame_count = 0

        # Store previous annotated frame
        self.last_result_frame = None

        try:
            from ultralytics import YOLO

            self.model = YOLO(model_name)

            self.enabled = True

            print(f"YOLOv8 detector loaded: {model_name}")

        except ImportError:

            print(
                "YOLOv8 detector disabled: "
                "install ultralytics to enable object detection."
            )

        except Exception as exc:

            print(f"YOLOv8 detector disabled: {exc}")

    def detect_and_draw(self, frame):

        if not self.enabled:
            return frame

        # -------- Run YOLO Every 5 Frames -------- #

        self.frame_count += 1

        if self.frame_count % 5 != 0:

            if self.last_result_frame is not None:
                return self.last_result_frame

            return frame

        temp_path = None

        try:

            # -------- Save Temporary Frame -------- #

            tf = tempfile.NamedTemporaryFile(
                suffix='.jpg',
                delete=False
            )

            temp_path = tf.name
            tf.close()

            cv2.imwrite(temp_path, frame)

            # -------- YOLO Inference -------- #

            results = self.model(
                temp_path,
                conf=self.confidence
            )

            # -------- Draw Detections -------- #

            for result in results:

                boxes = result.boxes

                if boxes is None:
                    continue

                names = result.names

                for box in boxes:

                    class_id = int(box.cls[0])

                    class_name = names[class_id]

                    if class_name not in self.TARGET_CLASSES:
                        continue

                    confidence = float(box.conf[0])

                    x1, y1, x2, y2 = (
                        box.xyxy[0]
                        .cpu()
                        .numpy()
                        .astype(int)
                    )

                    color = (0, 255, 0)

                    cv2.rectangle(
                        frame,
                        (x1, y1),
                        (x2, y2),
                        color,
                        2
                    )

                    label = f"{class_name} {confidence:.2f}"

                    cv2.putText(
                        frame,
                        label,
                        (x1, max(y1 - 8, 16)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2
                    )

            # Save latest annotated frame
            self.last_result_frame = frame.copy()

        except Exception as exc:

            print(f"YOLO detection error: {exc}")

        finally:

            if temp_path is not None:
                try:
                    os.remove(temp_path)
                except Exception:
                    pass

        return frame