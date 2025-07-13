from ultralytics import YOLO


# Modell laden
model = YOLO("src/senv/senv/best3.0.pt")  # z. B. "./yolo_model/best.pt"

# Bildpfad (ersetzen durch dein Bild)
img_path = "src/sign_recognition/test.jpg"

# Vorhersage (Inferenz)
results = model(img_path)  # kann auch save=True sein

# Klassenliste aus dem Modell (die Namen müssen mit deinem `data.yaml` übereinstimmen!)
class_names = model.names

# IDs der erkannten Klassen (z. B. 0, 1, 2 …)
class_ids = results[0].boxes.cls.cpu().numpy().astype(int)

# Umwandeln in Namen
detected_labels = [class_names[i] for i in class_ids]

print("Erkannte Schilder:", detected_labels[0])
