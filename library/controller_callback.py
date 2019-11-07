"""
Python file to interface the XBOX controller with python functions


USeful information: https://www.stuffaboutcode.com/2014/10/raspberry-pi-xbox-360-controller-python.html
"""
import XboxController


"""
Example function for what happens on start button press
"""
def startButtonCallback(value):
    print("Start button pressed / released")
    print("value is: ", value)


def triggerCallback(trigger, value):
    print(trigger, " value is: ", value)


def joystickCallback(joystick, axis, value):
    print(joystick, " ", axis, " value is: ", value)

"""
Exits out of the code using the back button
"""
def backButtonCallback(value):
    xboxCont.stop()


"""
Defines and instance of XboxController and the callback functions for the start and back button
"""
xboxCont = XboxController.XboxController(
    controllerCallBack = None,
    joystickNo = 0,
    deadzone = 0.15,
    scale = 1,
    invertYAxis = False)

xboxCont.setupControlCallback(
        xboxCont.XboxControls.START,
        startButtonCallback)

xboxCont.setupControlCallback(
xboxCont.XboxControls.BACK,
        backButtonCallback)

xboxCont.setupControlCallback(
    xboxCont.XboxControls.RTRIGGER,
    lambda value: triggerCallback("right trigger", value))

xboxCont.setupControlCallback(
    xboxCont.XboxControls.LTRIGGER,
    lambda value: triggerCallback("left trigger", value))

xboxCont.setupControlCallback(
    xboxCont.XboxControls.LTHUMBX,
    lambda value: joystickCallback("left joystick", 0, value))

xboxCont.start()
