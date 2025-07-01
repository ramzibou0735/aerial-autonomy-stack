import cv2
import numpy as np
import onnxruntime as ort

# Load model
sess = ort.InferenceSession("yolov8n.onnx", providers=["CUDAExecutionProvider", "CPUExecutionProvider"])
input_name = sess.get_inputs()[0].name
input_shape = sess.get_inputs()[0].shape  # [1, 3, 640, 640]

# Load video
cap = cv2.VideoCapture("/sample.mp4")
assert cap.isOpened(), "Failed to open video"

CONF_THRESH = 0.25
IOU_THRESH = 0.45
INPUT_SIZE = 640

def xywh2xyxy(x):
    """Convert [x_center, y_center, w, h] to [x1, y1, x2, y2]"""
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2  # x1
    y[..., 1] = x[..., 1] - x[..., 3] / 2  # y1
    y[..., 2] = x[..., 0] + x[..., 2] / 2  # x2
    y[..., 3] = x[..., 1] + x[..., 3] / 2  # y2
    return y

def nms(boxes, scores, iou_threshold):
    """Pure Python NMS for simplicity"""
    indices = cv2.dnn.NMSBoxes(
        bboxes=boxes.tolist(), scores=scores.tolist(), score_threshold=CONF_THRESH, nms_threshold=iou_threshold
    )
    return indices.flatten() if len(indices) > 0 else []

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h0, w0 = frame.shape[:2]

    # Preprocess
    img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_rgb = img_rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
    img_input = np.expand_dims(img_rgb, axis=0)

    # Inference
    outputs = sess.run(None, {input_name: img_input})
    preds = outputs[0]  # [1, N, 85] — for YOLOv8

    # Postprocessing
    preds = preds[0]
    boxes = preds[:, :4]
    scores = preds[:, 4]
    class_scores = preds[:, 5:]

    confs = scores * np.max(class_scores, axis=1)
    cls_ids = np.argmax(class_scores, axis=1)

    # Filter by confidence
    mask = confs > CONF_THRESH
    boxes, confs, cls_ids = boxes[mask], confs[mask], cls_ids[mask]

    # Convert boxes to xyxy and scale to original size
    boxes = xywh2xyxy(boxes)
    # boxes *= np.array([w0 / INPUT_SIZE, h0 / INPUT_SIZE, w0 / INPUT_SIZE, h0 / INPUT_SIZE])
    scale_w, scale_h = w0 / INPUT_SIZE, h0 / INPUT_SIZE
    boxes[:, [0, 2]] *= scale_w  # x1, x2
    boxes[:, [1, 3]] *= scale_h  # y1, y2

    # Run NMS
    if len(boxes) > 0:
        keep = nms(boxes.astype(np.int32), confs, IOU_THRESH)
        for i in keep:
            x1, y1, x2, y2 = boxes[i].astype(int)
            label = int(cls_ids[i])
            conf = confs[i]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"ID {label} {conf:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Show result
    cv2.imshow("YOLOv8 ONNX Inference", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

