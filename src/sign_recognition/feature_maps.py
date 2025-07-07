import torch
import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO

# Modell laden
model = YOLO(r"/home/lennart/ros2_ws/src/senv/senv/best3.0.pt")
print(f" Anzahl Layer : {len(list(model.model.model))}")  # Anzahl Layer im Modell
print(f" Anzahl Sub-Layer : {len(list(model.model.modules()))}")  # Anzahl Sub-Layer im Modell (Layer in Conv2d, BatchNorm, etc.)
# Speicher für Feature Maps
feature_maps = {}


# Hook-Funktion
def save_hook(module, input, output):
    feature_maps[module.layer_name] = output
    print(type(module))


# Gewünschte Layer aus YOLO-Modell (Layer 13, 17, 20 laut Diagramm = P3, P4, P5)
# kann auch zu anderen Layern geändert werden (1-20)
# nach layer 20 können die Feature Maps nicht mehr visualisiert werden
# es darf nur eine gerade anzahl layer sein, da sonst die visualisierung nicht mehr funktioniert oder angepasst werden muss
layers_to_hook = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
print(type(model))
# Hook registrieren
for i in layers_to_hook:
    layer = model.model.model[i]
    layer.layer_name = f"layer_{i}"
    layer.register_forward_hook(save_hook)

# Bild vorbereiten
img_path = r"src/sign_recognition/right.jpg"
img = cv2.imread(img_path)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_resized = cv2.resize(img_rgb, (640, 640))
img_tensor = torch.from_numpy(img_resized).permute(2, 0, 1).float().unsqueeze(0) / 255.0
# Modell durchlaufen lassen (liefert trotzdem normale Detect-Ausgabe)
_ = model.model(img_tensor)

# Visualisieren der gespeicherten Feature Maps
f_img = []
for idx, key in enumerate(feature_maps.keys()):
    fmap = feature_maps[key].squeeze().detach().cpu()
    fmap_img = fmap[0]  # Kanal 0
    np_img = fmap_img.numpy()
    np_img = cv2.normalize(np_img, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
    np_img = cv2.resize(np_img, (150, 150))
    f_img.append(np_img)

first_half = cv2.hconcat(f_img[:10])
second_half = cv2.hconcat(f_img[10:20])
# schwarz und weiß feature Maps in 200x200 auflösung untereinander
combined_img = cv2.vconcat([first_half, second_half])
colored = cv2.applyColorMap(combined_img, cv2.COLORMAP_VIRIDIS)
cv2.imshow("", colored)
cv2.waitKey(0)
cv2.destroyAllWindows()
