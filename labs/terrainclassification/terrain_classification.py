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

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here
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
    # Next, process the image to ensure the CNN is classifying only the terrain in the image


    #The following two lines save the image as a .jpg in the current working directory
    image = Image.fromarray(camera_image)
    image.save("./currentCameraImage.jpg")

    # TODO (main challenge): Have the CNN process the image and print the terrain being driven on
    with torch.no_grad():
        #The following three lines open the saved image, apply the test transforms, and perform additional pre-processing to the image.
        image = Image.open("./currentCameraImage.jpg")
        image_tensor = test_transform(image).float()
        image_tensor = image_tensor.unsqueeze_(0)

    # TODO (main challenge): Adjust the speed of the car based on the terrain (rougher terrains - lower speed, smoother terrains - higher speed)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
