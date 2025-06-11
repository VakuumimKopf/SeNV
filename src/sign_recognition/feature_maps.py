import torch
import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO

# Modell laden
model = YOLO(r"src/senv/best3.0.pt")
print(f" Anzahl Layer : {len(list(model.model.model))}")  # Anzahl Layer im Modell
print(f" Anzahl Sub-Layer : {len(list(model.model.modules()))}")  # Anzahl Sub-Layer im Modell (Layer in Conv2d, BatchNorm, etc.)
# Speicher für Feature Maps
feature_maps = {}


# Hook-Funktion
def save_hook(module, input, output):
    feature_maps[module.layer_name] = output


# Gewünschte Layer aus YOLO-Modell (Layer 13, 17, 20 laut Diagramm = P3, P4, P5)
# kann auch zu anderen Layern geändert werden (1-20)
# nach layer 20 können die Feature Maps nicht mehr visualisiert werden
layers_to_hook = [1, 2, 20]

# Hook registrieren
for i in layers_to_hook:
    layer = model.model.model[i]
    layer.layer_name = f"layer_{i}"
    layer.register_forward_hook(save_hook)

# Bild vorbereiten
img_path = r"src/sign_recognition/left.jpg"
img = cv2.imread(img_path)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_resized = cv2.resize(img_rgb, (640, 640))
img_tensor = torch.from_numpy(img_resized).permute(2, 0, 1).float().unsqueeze(0) / 255.0

# Modell durchlaufen lassen (liefert trotzdem normale Detect-Ausgabe)
_ = model.model(img_tensor)

# Visualisieren der gespeicherten Feature Maps
plt.figure(figsize=(12, 4))
for idx, key in enumerate(sorted(feature_maps.keys())):
    fmap = feature_maps[key].squeeze().detach().cpu()
    fmap_img = fmap[0]  # Kanal 0
    plt.subplot(1, 3, idx+1)
    plt.imshow(fmap_img, cmap="viridis")
    plt.title(f"{key}")
    plt.axis("off")
plt.tight_layout()
plt.show()
