#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node

import cv2
from ultralytics import YOLO

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("engine_path", "/home/justin/yolo26n.engine")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("publish_image", True)
        self.declare_parameter("fps", 15.0)

        engine_path   = self.get_parameter("engine_path").get_parameter_value().string_value
        camera_index  = self.get_parameter("camera_index").get_parameter_value().integer_value
        self.conf_thresh   = self.get_parameter("confidence_threshold").get_parameter_value().double_value
        publish_image      = self.get_parameter("publish_image").get_parameter_value().bool_value
        fps                = self.get_parameter("fps").get_parameter_value().double_value

        self.get_logger().info(f"Loading engine: {engine_path}")
        self.model = YOLO(engine_path, task="detect")
        self.get_logger().info("Engine loaded")

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().fatal(f"Cannot open camera index {camera_index}")
            raise RuntimeError("Camera not available")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.get_logger().info(f"Camera /dev/video{camera_index} opened")

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(Detection2DArray, "detections", 10)
        self.img_pub = self.create_publisher(Image, "camera/image_raw", 10) if publish_image else None

        self._frame      = None
        self._frame_lock = threading.Lock()
        self._shutting_down = False

        # Guard: drop timer tick if previous inference is still running
        self._inferring    = False
        self._infer_lock   = threading.Lock()

        self._cap_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="cam_capture")
        self._cap_thread.start()

        self.timer = self.create_timer(1.0 / fps, self._inference_callback)

    # ── Capture loop ──────────────────────────────────────────────────────
    def _capture_loop(self):
        while not self._shutting_down:
            ret, frame = self.cap.read()
            if ret:
                with self._frame_lock:
                    self._frame = frame

    # ── Inference callback ────────────────────────────────────────────────
    def _inference_callback(self):
        if self._shutting_down or not self.context.ok():
            return

        # Skip this tick if inference from the last tick is still running
        with self._infer_lock:
            if self._inferring:
                return
            self._inferring = True

        try:
            with self._frame_lock:
                if self._frame is None:
                    return
                frame = self._frame.copy()

            now     = self.get_clock().now().to_msg()
            results = self.model(frame, conf=self.conf_thresh, verbose=False)

            det_array = Detection2DArray()
            det_array.header.stamp    = now
            det_array.header.frame_id = "camera"

            for result in results:
                for box in result.boxes:
                    det = Detection2D()
                    det.header = det_array.header

                    xyxy = box.xyxy[0].cpu().numpy()
                    cx   = float((xyxy[0] + xyxy[2]) / 2.0)
                    cy   = float((xyxy[1] + xyxy[3]) / 2.0)
                    w    = float(xyxy[2] - xyxy[0])
                    h    = float(xyxy[3] - xyxy[1])

                    det.bbox.center.position.x = cx
                    det.bbox.center.position.y = cy
                    det.bbox.size_x = w
                    det.bbox.size_y = h

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(int(box.cls[0].item()))
                    hyp.hypothesis.score    = float(box.conf[0].item())
                    det.results.append(hyp)
                    det_array.detections.append(det)

            if not self.context.ok():
                return

            self.det_pub.publish(det_array)

            if self.img_pub is not None:
                annotated = results[0].plot()
                msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                msg.header.stamp    = now
                msg.header.frame_id = "camera"
                self.img_pub.publish(msg)

            if det_array.detections:
                self.get_logger().info(
                    f"{len(det_array.detections)} detection(s): " +
                    ", ".join(
                        f"cls={d.results[0].hypothesis.class_id} "
                        f"conf={d.results[0].hypothesis.score:.2f}"
                        for d in det_array.detections
                    )
                )
        finally:
            with self._infer_lock:
                self._inferring = False

    def destroy_node(self):
        self._shutting_down = True
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
