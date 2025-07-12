import os
import cv2
import json
import numpy as np
import onnxruntime as ort

# Load classes
names_file = "coco.json"
with open(names_file, "r") as f:
    classes_str_keys = json.load(f)
    classes = {int(k): v for k, v in classes_str_keys.items()}

# Load model runtime
model_path = "yolov8n.onnx"
session = ort.InferenceSession(model_path, providers=["CUDAExecutionProvider"])
# session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
# session = ort.InferenceSession(model_path, providers=["TensorRTExecutionProvider"])
input_name = session.get_inputs()[0].name

# Confirm execution providers
print("Execution providers in use:", session.get_providers())

# Acquire video stream
gst_pipeline_string = (
    "udpsrc port=5600 ! "
    "application/x-rtp, media=(string)video, encoding-name=(string)H264 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! "
    "video/x-raw, format=BGR ! appsink"
)
cap = cv2.VideoCapture(gst_pipeline_string, cv2.CAP_GSTREAMER) # From command line $ gst-launch-1.0 udpsrc port=5600 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
# cap = cv2.VideoCapture("sample.mp4") # Load example video
assert cap.isOpened(), "Failed to open video"

drone_id = os.getenv('DRONE_ID', '0')
WINDOW_NAME = f"YOLOv8 (Aircraft {drone_id})"
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.moveWindow(WINDOW_NAME, 1500-(int(drone_id)-1)*50, 5+(int(drone_id)-1)*150)
# cv2.resizeWindow(WINDOW_NAME, 400, 200)

def xywh2xyxy(box):
    """Convert [x, y, w, h] to [x1, y1, x2, y2]"""
    coord = np.copy(box)
    coord[..., 0] = box[..., 0] - box[..., 2] / 2  # x1
    coord[..., 1] = box[..., 1] - box[..., 3] / 2  # y1
    coord[..., 2] = box[..., 0] + box[..., 2] / 2  # x2
    coord[..., 3] = box[..., 1] + box[..., 3] / 2  # y2
    return coord

while True:
    ret, frame = cap.read()
    if not ret:
        break
    h0, w0 = frame.shape[:2]

    # Preprocess fram
    INPUT_SIZE = 640 # YOLOv8 input size
    img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)

    # Inference
    outputs = session.run(None, {input_name: img})
    raw_output = outputs[0] # shape: [1, 84, 8400]
    raw_output = np.squeeze(raw_output) # shape: [84, 8400]
    raw_output = raw_output.transpose(1, 0) # shape: [8400, 84]
    
    # Separate boxes and class scores
    preds = raw_output # shape: [8400, 84]
    boxes = preds[:, :4]
    class_scores = preds[:, 4:] # shape: [8400, 80]
    class_ids = np.argmax(class_scores, axis=1)
    confidences = np.max(class_scores, axis=1)

    # Filter with confidence threshold
    CONF_THRESH = 0.3
    mask = (confidences > CONF_THRESH)
    boxes = boxes[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]

    # Convert boxes to xyxy and rescale to original size
    boxes = xywh2xyxy(boxes)
    scale_w, scale_h = w0 / INPUT_SIZE, h0 / INPUT_SIZE
    boxes[:, [0, 2]] *= scale_w
    boxes[:, [1, 3]] *= scale_h

    # Draw boxes
    for i in range(len(boxes)):
         #print(f"Detected: Class {class_ids[i]}, Confidence {confidences[i]:.2f}, Box {boxes[i]}")
        x1, y1, x2, y2 = boxes[i].astype(int)
        conf = confidences[i]
        class_name = classes[class_ids[i]]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    # Display
    cv2.imshow(WINDOW_NAME, frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
