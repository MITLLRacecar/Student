"""
Python file to interface the XBOX controller with python functions


USeful information: https://www.stuffaboutcode.com/2014/10/raspberry-pi-xbox-360-controller-python.html
"""
import XboxController


"""
Example function for what happens on start button press
"""
def startButtonCallback(value):
    print "Start button pressed / released"


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
    deadzone = 0.1,
    scale = 1,
    invertYAxis = False)

xboxCont.setupControlCallback(
        xboxCont.XboxControls.START,
        startButtonCallback)

xboxCont.setupControlCallback(
xboxCont.XboxControls.BACK,
        backButtonCallback)

xboxCont.start()
