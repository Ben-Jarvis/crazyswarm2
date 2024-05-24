import os
from PIL import Image

input_folder = '/home/jarvis/ros2_ws/output_20240208-190338'
output_file = '/home/jarvis/ros2_ws/four_back_0208_1903.gif'
duration = 125

images = []
image_files = sorted([f for f in os.listdir(input_folder)])

for image_file in image_files:
    image_path = os.path.join(input_folder, image_file)
    with Image.open(image_path) as img:
        images.append(img.copy())

images[0].save(output_file, save_all=True, append_images=images[1:], duration=duration, loop=0)