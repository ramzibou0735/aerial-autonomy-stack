import cv2
import numpy as np
import onnxruntime as ort

# Load model
model_path = "yolov8n.onnx"
session = ort.InferenceSession(model_path, providers=["CUDAExecutionProvider", "CPUExecutionProvider"])
input_name = session.get_inputs()[0].name

# Load video
cap = cv2.VideoCapture("sample.mp4")
assert cap.isOpened(), "Failed to open video"

CONF_THRESH = 0.3
INPUT_SIZE = 640

def xywh2xyxy(x):
    """Convert [x, y, w, h] to [x1, y1, x2, y2]"""
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2  # x1
    y[..., 1] = x[..., 1] - x[..., 3] / 2  # y1
    y[..., 2] = x[..., 0] + x[..., 2] / 2  # x2
    y[..., 3] = x[..., 1] + x[..., 3] / 2  # y2
    return y

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h0, w0 = frame.shape[:2]

    # Preprocess
    img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)

    # Inference
    outputs = session.run(None, {input_name: img})
    raw_output = outputs[0]  # shape: [1, 84, 8400]
    raw_output = np.squeeze(raw_output)           # shape: [84, 8400]
    raw_output = raw_output.transpose(1, 0)       # shape: [8400, 84]
    
    preds = raw_output  # shape: [8400, 84]
    boxes = preds[:, :4]
    class_scores = preds[:, 4:]  # shape: [8400, 80]
    class_ids = np.argmax(class_scores, axis=1)
    confidences = np.max(class_scores, axis=1)

    # Filter with confidence threshold
    mask = (confidences > CONF_THRESH)
    boxes = boxes[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]

    # Convert boxes to xyxy and scale to original size
    boxes = xywh2xyxy(boxes)
    scale_w, scale_h = w0 / INPUT_SIZE, h0 / INPUT_SIZE
    boxes[:, [0, 2]] *= scale_w
    boxes[:, [1, 3]] *= scale_h

    # Draw boxes
    for i in range(len(boxes)):
        print(f"Detected: Class {class_ids[i]}, Confidence {confidences[i]:.2f}, Box {boxes[i]}")
        x1, y1, x2, y2 = boxes[i].astype(int)
        conf = confidences[i]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"Car {conf:.2f}", (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    # Display
    cv2.imshow("YOLOv8 Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
