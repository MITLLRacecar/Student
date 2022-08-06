"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2022

Bonus Lab - Terrain Classification
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

#Put any additional import statements that are required here
import cv2 as cv
import torch
import torch.nn as nn
import torchvision.transforms as transforms
from torch.autograd import Variable
from PIL import Image
import PIL
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here
model = models.resnet18(pretrained = True)
model.fc = torch.nn.Linear(512, 4)
model = model.eval()

test_transform = transforms.Compose([transforms.RandomResizedCrop(224),
                                     transforms.CenterCrop(224),
                                     transforms.ToTensor(),
                                     transforms.Normalize((0.485,0.456,0.406), (0.229,0.224,0.225))
                                   ])

terrain_dictionary = {0: "Asphalt", 1: "Dirt", 2: "Grass", 3: "Rocky"}

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """

    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    print(">> Bonus Lab - Terrain Classification\n")

    #This line loads the pre-trained ResNet-18 CNN model
    model.load_state_dict(torch.load("TerrainNet.pt"))


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    #Control the car's steering angle with the left joystick or "A" and "D" keys
    (x, y) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)

    # TODO (warmup): Capture an image (the camera_image variable) from the color camera every second
    # Next, process the image to ensure the CNN is classifying only the terrain
    if counter % 60 == 0:
        camera_image = rc.camera.get_color_image()
        camera_image = rc_utils.crop(camera_image, (rc.camera.get_height() // 2, 0), (rc.camera.get_height(), rc.camera.get_width()))
        camera_image = cv.cvtColor(camera_image, cv.COLOR_BGR2RGB)

        #The following two lines save the image as a .jpg in the current working directory
        image = Image.fromarray(camera_image)
        image.save("./currentCameraImage.jpg")


    # TODO (main challenge): Have the CNN process the image and print the terrain being driven on
        with torch.no_grad():
            #The following three lines open the saved image, apply the test transforms, and perform additional pre-processing to the image.
            image = Image.open("./currentCameraImage.jpg")                                 #Open the image taken from above
            image_tensor = test_transform(image).float()                                   #Perform all transforms on the image
            image_tensor = image_tensor.unsqueeze_(0)                                      #Add another filler dimension of size 1

            model_output = model(Variable(image_tensor))                                   #Pass the image into the model
            _, predicted_category = torch.max(model_output.data, 1)                        #Process the predicted category
            predicted_category = predicted_category.float()                                #Convert the predicted category to a float

        terrain = terrain_dictionary.get(predicted_category.tolist()[0])                   #Find the corresponding terrain from the terrain dictionary

    # TODO (main challenge): Adjust the speed of the car based on the terrain (rougher terrains - lower speed, smoother terrains - higher speed)
        if terrain == "Asphalt":
            speed = 1.0
            print("Asphalt: MAX SPEED\n\n")
        elif terrain == "Grass":
            speed = 0.75
            print("Grass: MODERATE SPEED\n\n")
        elif terrain == "Dirt":
            speed = 0.5
            print("Dirt: MODERATE SPEED\n\n")
        elif terrain == "Rocky":
            speed = 0.25
            print("Rocky: LOW SPEED\n\n")

    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
