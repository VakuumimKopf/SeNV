'''
Add the following Code to camera, to create a video of the 20 feature maps
which come from 20 out of 23 neurons (main layers) of our sign-recognition model

Frame rate
'''

# variable for feature map generation
# have to be added to def __init__

self.feature_maps = {}
self.frames = 300
# fps do not represent the real time fps
fps = 40
self.video = cv2.VideoWriter('maps.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (1500, 300))


# this entire part has to be added to the class
def save_hook(self, module, input, output):
    self.feature_maps[module.layer_name] = output

def feature_video(self):
    if self.raw_image is None:
        return ""
    self.feature_maps = {}
    layers_to_hook = []
    # n is the number of layers which will be shown
    n = 20
    for i in range(1, n+1):
        layers_to_hook.append(i)
    for i in layers_to_hook:
        layer = model.model.model[i]
        layer.layer_name = f"layer_{i}"
        layer.register_forward_hook(self.save_hook)
    img = self.raw_image
    img_resized = cv2.resize(img, (640, 640))
    img_tensor = torch.from_numpy(img_resized).permute(2, 0, 1).float().unsqueeze(0) / 255.0
    _ = model.model(img_tensor)
    f_img = []
    for idx, key in enumerate(self.feature_maps.keys()):
        fmap = self.feature_maps[key].squeeze().detach().cpu()
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
    self.video.write(colored)
    cv2.imshow("", colored)
    cv2.waitKey(1)
    self.frames -= 1
    self.get_logger().info(f"Frame aufgenommen {self.frames}")
    if self.frames == 0:
        self.video.release()
        self.get_logger().info("Video gespeichert")
        cv2.destroyAllWindows()
