import torch
from torchvision import datasets, models, transforms
import os
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np

input_size = 224

data_transforms = {
    'train': transforms.Compose([
        transforms.RandomResizedCrop(input_size),
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]),
    'val': transforms.Compose([
        transforms.Resize(input_size),
        transforms.CenterCrop(input_size),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ]),
}

class_dict = {0:'normal_face', 1:'wanted_poster'}

images_path = "/home/aljaz/ROS/src/task3/scripts/all_pictures"
model_path = '/home/aljaz/ROS/src/task3/scripts/training_posters/best_posters_model.pt'

model = torch.load(model_path)
model.eval()

for image_name in os.listdir(images_path):
    image_file = os.path.join(images_path, image_name)

    img_p = Image.open(image_file)

    if img_p.mode == 'RGBA':
        img_p = img_p.convert('RGB')

    img = data_transforms['train'](img_p).unsqueeze(0)
    pred = model(img)

    pred_np = pred.cpu().detach().numpy().squeeze()
    class_ind = np.argmax(pred_np)

    if 'poster' in image_name and class_ind == 1:
        # print text in green in terminal
        print("\033[92m" + f"Image:{image_name} Class:{class_dict[class_ind]}" + "\033[0m")
    elif 'face' in image_name and class_ind == 0:
        print("\033[92m" + f"Image:{image_name} Class:{class_dict[class_ind]}" + "\033[0m")
    # print wrong classificarion in red in terminal
    else:
        print("\033[91m" + f"Image:{image_name} Class:{class_dict[class_ind]}" + "\033[0m")

    # print(f"Class index: {class_ind} Prob: {pred_np}")


