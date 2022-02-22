"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

"""

"""
- In this file, you can specify the xy coordinates of the destination the car should drive to.
- The car will then drive to those coordinates while adjusting its speed based on the terrain as well as how close it is to the destination
- It is a bit more consistent when the realism setting is off.
"""

########################################################################################
# Imports
########################################################################################
import sys
sys.path.insert(0, "../library")
import racecar_core
import racecar_utils as rc_utils
import numpy as np
import math
import cv2 as cv
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as transforms
from torchvision import datasets, models
from torch.autograd import Variable
from matplotlib import pyplot as plt
from PIL import Image
import PIL
import os

########################################################################################
# Global variables
########################################################################################
rc = racecar_core.create_racecar()
startingPosition = np.array([0, 0])
destinationPosition = np.array([-87, 34]) #CHANGE THIS TO THE DESIRED DEST. COORDS!
speed = 0

########################################################################################
# Functions
########################################################################################
def start():
    """
    This function is run once every time the start button is pressed
    """
    #Navigation Global Variables:
    global iter
    global currPosition
    global prevPosition
    global realism
    global speedMultiplier

    #CNN Global Variables
    global counter
    global model
    global image
    global test_loader
    global batch_size
    global num_workers
    global test_transform
    global finalTerrainDict
    global unCropped
    global cropped
    global terrain

    #Initialize navigation variables
    iter = 0
    currPosition = startingPosition
    realism = True
    speedMultiplier = multiplierRegression(np.linalg.norm(destinationPosition)) if realism == True else 1.0

    #Create the pretrained ResNet18 model here and load it in
    model = models.resnet18(pretrained=True)
    model.fc = torch.nn.Linear(512, 4)
    model = model.eval()                                                                              #Put the model into validation/test mode and not train mode
    model.load_state_dict(torch.load("Terrain_ResNet18_BW_Implementation_Unity_Terrains_FINAL.pt"))   #Unity Model trained on FinalDataSet

    #Initialize CNN variables
    batch_size = 1
    num_workers = 0
    counter = 0
    terrain = "INIT"
    finalTerrainDict = {0: "Asphalt", 1: "Dirt", 2: "Grass", 3: "Rocky"}                                        #Unity Model trained on FinalDataSet

    # Define the inferencing transforms:
    test_transform = transforms.Compose([transforms.RandomResizedCrop(224),
                                        transforms.CenterCrop(224),
                                        transforms.ToTensor(),
                                        transforms.Normalize((0.485,0.456,0.406), (0.229,0.224,0.225))
                                        ])

    # Begin at a full stop
    rc.drive.stop()


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    #Navigation Global variables
    global iter
    global currPosition
    global prevPosition
    global realism
    global speedMultiplier

    #CNN Global Variables
    global counter
    global image
    global test_loader
    global unCropped
    global cropped
    global terrain


    """
    CAR NAVIGATION BEGINS HERE
    """
    #Determine current and desired heading by referencing the final position and the previous position
    if iter == 0:
        targetVector = (destinationPosition - currPosition)
        velocityVector = np.array([0.00, 10.00])
    else:
        targetVector = (destinationPosition - currPosition)
        velocityVector = (currPosition - prevPosition)

    angle = calculateAngle(velocityVector, targetVector)

    #printNavigationStats()

    #Update current position
    prevPosition = currPosition
    currPosition = np.array([-rc.physics.get_position()[0], -rc.physics.get_position()[2]])

    #Use P control to adjust speed based on error
    error = calculateError(currPosition)
    speed = rc_utils.remap_range(error, 0, speedMultiplier * math.sqrt(((destinationPosition[0] - startingPosition[0]) ** 2) + ((destinationPosition[1] - startingPosition[1]) ** 2)), 0, 0.6)

    #Stop the car if you are within a ~5m threshold to the destination
    if (math.floor(error) <= 5):
        rc.drive.stop()
        speed = 0
        angle = 0


    """
    CNN TERRAIN IDENTIFICATION BEGINS HERE
    """
    color_image = rc.camera.get_color_image()

    #Sample images and speed/angle data
    if iter % 15 == 0:
      captureAndSaveImage()
      terrain = finalTerrainDict.get(makePrediction()[0])
      print(f"Terrain: {terrain}")
      #print("Original speed => {}".format(speed))
      speed = 0.2 * speed + 0.8 * displayCNNResults(terrain, speed)
      #print("New speed => {}".format(speed))
      counter = 0

      #Error and Position stats will be printed here along with the terrain prediction above
      #print("Error => {}".format(math.floor(error)))
      print("Current (X,Y) => ({}, {})".format(-rc.physics.get_position()[0], -rc.physics.get_position()[2]))
      print("Target (X,Y) => ({}, {})\n".format(destinationPosition[0], destinationPosition[1]))
      rc.drive.set_speed_angle(speed, angle)

    iter += 1


#Capture current color camera image and save for future processing
def captureAndSaveImage():
    unCropped = rc.camera.get_color_image()
    cropped = rc_utils.crop(unCropped, (rc.camera.get_height() // 2, 0), (rc.camera.get_height(), rc.camera.get_width()))
    image = cv.cvtColor(cropped, cv.COLOR_BGR2RGB)              #The camera gets images in BGR so you need to convert it to RGB for the model to process
    im = Image.fromarray(image)
    im.save("./currentCameraImage.jpg")


#Feed image into CNN and get terrain prediction
def makePrediction():
    #This is for preparing the image and passing it into the model
    with torch.no_grad():
        image = Image.open("./currentCameraImage.jpg")                                 #Open the image taken from above
        imageTensor = test_transform(image).float()                                    #Perform all transforms on the image
        imageTensor = imageTensor.unsqueeze_(0)                                        #Add another filler dimension with a value of 1 for the batch parameter

        outputT = model(Variable(imageTensor))                                         #Pass the image into the model
        predictedCategory = outputT.data                                               #Get the predicted category/terrain
        _, predictedCategory = torch.max(outputT.data, 1)                              #Process the predicted category
        predictedCategory = predictedCategory.float()                                  #More processing of the predicted category
        #print("predictions", predictedCategory.tolist())                              #Print out the predicted category as a list
    return predictedCategory.tolist()


#Prints the terrain prediction and the speed recommendation
def displayCNNResults(terrain, speed):
    if terrain == "Asphalt":
      print("SMOOTH TERRAIN: MAX SPEED")
    elif terrain == "Grass":
      speed = 0.4
      print("SMOOTH TERRAIN: MODERATE SPEED")
    elif terrain == "Dirt":
      speed = 0.2
      print("UNEVEN TERRAIN: MODERATE SPEED")
    elif terrain == "Rocky":
      speed = 0.1
      print("VERY UNEVEN TERRAIN: LOW SPEED")
    return speed


#Uses linear regression to determine multiplier for the car's speed given the total distance that needs to be traveled
def multiplierRegression(norm):
    #Equation sourced from curve fitting tool
    return -0.0035 * norm + 1.20


#Determines the angle the wheels need to turn given the current heading and the desired heading (uses dot product and determinant)
def calculateAngle(velocityVector, targetVector):
    dotProduct = np.dot(velocityVector, targetVector)
    normProduct = np.linalg.norm(velocityVector) * np.linalg.norm(targetVector)

    #Find the angle between the two vectors
    theta = math.degrees(math.acos(dotProduct / normProduct))
    theta = rc_utils.clamp(theta, -90, 90)

    #Invert if a left turn is required
    if np.linalg.det([velocityVector, targetVector]) > 0:
        theta = -theta

    angle = rc_utils.remap_range(theta, -90, 90, -1, 1)
    angle = rc_utils.clamp(angle, -1, 1)
    return angle


#Returns the Euclidean distance between the current position and the destination
def calculateError(currentPosition):
    xError = abs(destinationPosition[0] - currentPosition[0])
    yError = abs(destinationPosition[1] - currentPosition[1])
    error = round(math.sqrt((xError ** 2) + (yError ** 2)), 1)
    return error


#Method that contains print statements to monitor different parameters
def printNavigationStats():
    print("Multiplier => {}".format(speedMultiplier))
    print("Target vector => {}".format(targetVector))
    print("Velocity vector => {}".format(velocityVector))
    print("Dot product => {}".format(dotProduct))
    print("Norm product => {}".format(normProduct))
    print("Theta => {}".format(theta))
    print("Angle => {}".format(angle))

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################
if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
