import os
import shutil
import random

# Parameters
input_img_dir = "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/yolo_dataset/images"       # folder where screenshotX.jpg are
input_lbl_dir = "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/yolo_dataset/labels"    # folder where screenshotX.txt are
output_dir = "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/dataset"
train_ratio = 0.7
val_ratio = 0.2
test_ratio = 0.1

# Output folders
splits = ['train', 'val', 'test']
for split in splits:
    os.makedirs(os.path.join(output_dir, 'images', split), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'labels', split), exist_ok=True)

# List and shuffle image filenames
all_images = [f for f in os.listdir(input_img_dir) if f.endswith(".jpg")]
all_images.sort()
random.shuffle(all_images)

# Compute split sizes
n_total = len(all_images)
n_train = int(n_total * train_ratio)
n_val = int(n_total * val_ratio)

train_imgs = all_images[:n_train]
val_imgs = all_images[n_train:n_train + n_val]
test_imgs = all_images[n_train + n_val:]

split_map = {
    'train': train_imgs,
    'val': val_imgs,
    'test': test_imgs
}

# Copy files
for split, images in split_map.items():
    for img in images:
        base = os.path.splitext(img)[0]
        lbl = base + ".txt"

        # Copy image
        shutil.copy(os.path.join(input_img_dir, img),
                    os.path.join(output_dir, 'images', split, img))

        # Copy label
        shutil.copy(os.path.join(input_lbl_dir, lbl),
                    os.path.join(output_dir, 'labels', split, lbl))

print(f"✔️ Dataset split complete: {n_train} train, {n_val} val, {n_total - n_train - n_val} test")

