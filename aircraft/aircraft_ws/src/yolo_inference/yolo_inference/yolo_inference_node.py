import rclpy
from rclpy.node import Node
import cv2
import json
import numpy as np
import onnxruntime as ort
import argparse
import os
import matplotlib.pyplot as plt
import threading
import queue
import time
import platform

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesis, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

frame_queue = queue.Queue(maxsize=2) # A queue to hold frames

def frame_capture_thread(cap, is_running):
    frame_count = 0
    start_time = time.time()
    while is_running.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
        try:
            frame_queue.put(frame, timeout=0.1)
            frame_count += 1
        except queue.Full:
            pass # Drop frame if the main thread is lagging
        if frame_count % 120 == 0:
            end_time = time.time()
            elapsed_time = end_time - start_time
            fps = frame_count / elapsed_time
            print(f"Frame Reception Rate: {fps:.2f} FPS")
            # Reset counters
            frame_count = 0
            start_time = time.time()

def xywh2xyxy(box):
    # Convert [x, y, w, h] to [x1, y1, x2, y2]
    coord = np.copy(box)
    coord[..., 0] = box[..., 0] - box[..., 2] / 2  # x1
    coord[..., 1] = box[..., 1] - box[..., 3] / 2  # y1
    coord[..., 2] = box[..., 0] + box[..., 2] / 2  # x2
    coord[..., 3] = box[..., 1] + box[..., 3] / 2  # y2
    return coord

class YoloInferenceNode(Node):
    def __init__(self, headless):
        super().__init__('yolo_inference_node')
        self.headless = headless
        self.architecture = platform.machine()
        
        # Load classes
        names_file = "/aas/yolo/coco.json"
        with open(names_file, "r") as f:
            classes_str_keys = json.load(f)
            self.classes = {int(k): v for k, v in classes_str_keys.items()}
        colors_rgba = plt.cm.hsv(np.linspace(0, 1, len(self.classes)))
        self.colors = (colors_rgba[:, [2, 1, 0]] * 255).astype(np.uint8) # From RGBA (0-1 float) to BGR (0-255 int)

        # Load model runtime
        model_path = "/aas/yolo/yolov8n.onnx" # Model options (from fastest to most accurate, <10MB to >100MB): yolov8n, yolov8s, yolov8m, yolov8l, yolov8x
        if self.architecture == 'x86_64':
            print("Loading CUDAExecutionProvider on AMD64 (x86) architecture.")
            self.session = ort.InferenceSession(model_path, providers=["CUDAExecutionProvider"]) # For simulation
        elif self.architecture == 'aarch64':
            print("Loading (with cache) TensorrtExecutionProvider on ARM64 architecture (Jetson).") # The first cache built takes ~3'
            cache_path = "/tensorrt_cache" # Mounted as volume by main_deploy.sh
            os.makedirs(cache_path, exist_ok=True)
            provider_options = {
                'trt_engine_cache_enable': True,
                'trt_engine_cache_path': cache_path,
            }
            self.session = ort.InferenceSession(
                model_path,
                providers=[('TensorrtExecutionProvider', provider_options)] # For deployment on Jetson Orin
            )
        else:
            print(f"Loading CPUExecutionProvider on an unknown architecture: {self.architecture}")
            self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"]) # Backup, not recommended
        self.input_name = self.session.get_inputs()[0].name
        
        # Confirm execution providers
        self.get_logger().info(f"Execution providers in use: {self.session.get_providers()}")
        
        # Create publishers
        self.detection_publisher = self.create_publisher(Detection2DArray, 'detections', 10)
        # self.image_publisher = self.create_publisher(Image, 'detections_image', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info("YOLO inference started.")

    def run_inference_loop(self):
        # Acquire video stream
        if self.architecture == 'x86_64':
            gst_pipeline_string = (
                "udpsrc port=5600 ! "
                "application/x-rtp, media=(string)video, encoding-name=(string)H264 ! "
                "rtph264depay ! "
                "avdec_h264 threads=4 ! " # Use CPU decoder, threads=0 for autodetection
                "videoconvert ! "
                "video/x-raw, format=BGR ! appsink"
            )
            # NOT WORKING: system Python's OpenCV has GStreamer but no CUDA support
            # TODO: build OpenCV from source to support both or use python3-gi gir1.2-gst-plugins-base-1.0 gir1.2-gstreamer-1.0
            # gst_pipeline_string = (
            #     "udpsrc port=5600 ! "
            #     "'application/x-rtp, media=(string)video, encoding-name=(string)H264' ! "
            #     "rtph264depay ! "
            #     "nvh264dec ! "       # Use the NVIDIA hardware decoder
            #     "cudadownload ! "    # Copy the frame from GPU to CPU memory
            #     "videoconvert ! "
            #     "video/x-raw, format=BGR ! appsink"
            # )
            cap = cv2.VideoCapture(gst_pipeline_string, cv2.CAP_GSTREAMER)
        elif self.architecture == 'aarch64':
            gst_pipeline_string = (
            "udpsrc port=5600 ! "
                "application/x-rtp, media=(string)video, encoding-name=(string)H264 ! "
                "rtph264depay ! "
                "h264parse ! "
                "nvv4l2decoder ! "     # Hardware Decoding: Uses the Orin's dedicated engine
                "nvvidconv ! "         # NVMM-to-CPU Memory Conversion
                "video/x-raw, format=I420 ! "
                "videoconvert ! "      # CPU Color Conversion: I420 to BGR
                "video/x-raw, format=BGR ! "
                "appsink drop=true max-buffers=1 "
            )
            # ALSO WORKING: CPU fallback for test/debug
            # gst_pipeline_string = (
            #     "udpsrc port=5600 ! "
            #     "application/x-rtp, media=(string)video, encoding-name=(string)H264 ! "
            #     "rtph264depay ! "
            #     "avdec_h264 ! "      # Generic CPU H.264 decoder
            #     "videoconvert ! "
            #     "video/x-raw, format=BGR ! appsink"
            # )
            cap = cv2.VideoCapture(gst_pipeline_string, cv2.CAP_GSTREAMER)
            # cap = cv2.VideoCapture("sample.mp4") # Load example video for testing
            # TODO: open CSI camera
            # gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1' ! nvvidconv ! nv3dsink -e
        assert cap.isOpened(), "Failed to open video stream"

        if not self.headless:
            drone_id = os.getenv('DRONE_ID', '0')
            self.WINDOW_NAME = f"YOLOv8 (Aircraft {drone_id})"
            cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.moveWindow(self.WINDOW_NAME, 800+(int(drone_id)-1)*25, 5+(int(drone_id)-1)*200)
            # cv2.resizeWindow(self.WINDOW_NAME, 400, 200)

        # Start the video capture thread
        is_running = threading.Event()
        is_running.set()
        thread = threading.Thread(target=frame_capture_thread, args=(cap, is_running))
        thread.start()

        inference_count = 0
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0) # This is only to get the simulation time from /clock

            try:
                frame = frame_queue.get(timeout=1) # Get the most recent frame from the queue
            except queue.Empty:
                self.get_logger().info("Frame queue is empty, is the stream running?")
                continue
            
            # Inference
            boxes, confidences, class_ids = self.run_yolo(frame)

            inference_count += 1
            if inference_count % 120 == 0:
                end_time = time.time()
                elapsed_time = end_time - start_time
                yolo_fps = inference_count / elapsed_time
                print(f"YOLO Inference Rate: {yolo_fps:.2f} FPS")
                # Reset counters
                inference_count = 0
                start_time = time.time()

            # Publish detections
            self.publish_detections(frame, boxes, confidences, class_ids)

            # Visualize
            if not self.headless:
                self.visualize(frame, boxes, confidences, class_ids)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        # Cleanup
        is_running.clear()
        thread.join()
        
        cap.release()
        if not self.headless:
            cv2.destroyAllWindows()

    def run_yolo(self, frame):
        h0, w0 = frame.shape[:2]
        INPUT_SIZE = 640 # YOLOv8 input size
        
        img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)

        outputs = self.session.run(None, {self.input_name: img})
        preds = np.squeeze(outputs[0]).transpose()

        boxes = preds[:, :4]
        class_scores = preds[:, 4:]
        
        class_ids = np.argmax(class_scores, axis=1)
        confidences = np.max(class_scores, axis=1)

        CONF_THRESH = 0.5
        mask = (confidences > CONF_THRESH)

        # Apply Non-Maximal Suppression
        NMS_THRESH = 0.45
        boxes_for_nms = xywh2xyxy(boxes[mask])
        indices = cv2.dnn.NMSBoxes(boxes_for_nms.tolist(), confidences[mask].tolist(), CONF_THRESH, NMS_THRESH)
        
        final_boxes = boxes[mask][indices]
        final_confidences = confidences[mask][indices]
        final_class_ids = class_ids[mask][indices]
        
        final_boxes = xywh2xyxy(final_boxes)
        scale_w, scale_h = w0 / INPUT_SIZE, h0 / INPUT_SIZE
        final_boxes[:, [0, 2]] *= scale_w
        final_boxes[:, [1, 3]] *= scale_h
        return final_boxes, final_confidences, final_class_ids

    def publish_detections(self, frame, boxes, confidences, class_ids):
        detection_array_msg = Detection2DArray()
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()
        detection_array_msg.header.frame_id = "camera_frame"

        for i in range(len(boxes)):
            x1, y1, x2, y2 = boxes[i]
            bbox = BoundingBox2D()
            bbox.center.position.x = float((x1 + x2) / 2.0)
            bbox.center.position.y = float((y1 + y2) / 2.0)
            bbox.size_x = float(x2 - x1)
            bbox.size_y = float(y2 - y1)

            hypothesis = ObjectHypothesis()
            hypothesis.class_id = str(self.classes[class_ids[i]])
            hypothesis.score = float(confidences[i])
            result = ObjectHypothesisWithPose()
            result.hypothesis = hypothesis
            
            detection = Detection2D() 
            detection.bbox = bbox
            detection.id = str(self.classes[class_ids[i]]) 
            detection.results.append(result)
            detection_array_msg.detections.append(detection)

        self.detection_publisher.publish(detection_array_msg)
        
        # if not self.headless:
        #     self.image_publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def visualize(self, frame, boxes, confidences, class_ids):
        for i in range(len(boxes)):
            x1, y1, x2, y2 = boxes[i].astype(int)
            conf = confidences[i]
            class_id = class_ids[i]
            class_name = self.classes[class_id]
            color = tuple(self.colors[class_id, [2, 1, 0]].tolist())
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
        cv2.imshow(self.WINDOW_NAME, frame)

def main(args=None):
    parser = argparse.ArgumentParser(description="YOLOv8 ROS2 Inference Node.")
    parser.add_argument('--headless', action='store_true', help="Run in headless mode.")
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    yolo_node = YoloInferenceNode(headless=cli_args.headless)
    yolo_node.run_inference_loop()
    
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
