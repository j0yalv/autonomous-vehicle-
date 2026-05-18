import cv2


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

        # -------- Convert BGR -> RGB -------- #

        rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        # -------- YOLO Inference -------- #

        results = self.model.predict(
            source=[rgb_frame],
            conf=self.confidence,
            verbose=False
        )

        # -------- Draw Detections -------- #

        for result in results:

            names = result.names

            for box in result.boxes:

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

        return frame
