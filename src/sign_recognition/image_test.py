from ultralytics import YOLO
import cv2


# load model
model = YOLO("senv/best3.0.pt")  # z. B. "./yolo_model/best.pt"

# image path
img_path = "sign_recognition/schild.jpg"
# left.jpg, right.jpg, straight.jpg, cross.jpg, park.jpg
# prediction/inference
results = model.predict(img_path)

# classlist of model must be same as in training
class_names = model.names

# id of detected objects
class_ids = results[0].boxes.cls.cpu().numpy().astype(int)

# transform class ids to class names
detected_labels = [class_names[i] for i in class_ids]

print("Erkannte Schilder:", detected_labels[0])
cv2.imwrite("Erkannte_Schilder.jpg", results[0].plot())
