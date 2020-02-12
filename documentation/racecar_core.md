racecar_core
============

File docstring


Variables
----------

This file does not define any variables


Functions
----------

This file does not define any top-level functions


Classes
----------

### class `Racecar()`

The core racecar module which contains several submodules which interface
with and control the different pieces of hardware

Methods:

#### def `__init__()`



#### def `get_delta_time()`

Returns the number of seconds elapsed in the previous frame

Output (float): The number of seconds between the start of the previous
frame and the start of the current frame

#### def `go()`

Idles in the main thread until the car program is exited
(START + END pressed simultaneously)

#### def `set_start_update(start, update)`

Sets the start and update functions used in user program mode

Inputs:
start (function): The function called once every time we enter
user program mode
update (function): The function called every frame in user program
modes

#### def `__handle_back(self)`

Handles when the BACK button is pressed by entering default drive mode

#### def `handle_exit(self)`

Handles when BACK and START are pressed together by exiting the program

#### def `default_start`

The start function for default drive mode

#### def `default_update`

The update function for default drive mode, which controls the car with
the triggers and left joystick

#### def `__handle_start(self)`

Handles when the START button is pressed by entering user program mode

#### def `__run(self)`

Calls the current update function (determined by the current mode) 
and update_modules functions once per frame

#### def `__update_modules`

Calls the update function on each module


### class Drive

Controls the car's movement by allowing the user to set the state 
associated with speed and turning

Methods:


#### def `__init__()`


#### def `set_speed_angle(self, speed, angle)`

Sets the speed at which the wheels turn and the angle of the front
wheels

Inputs:
speed (float) = the speed from -1 to 1, with positive for 
forward and negative for reverse
angle (float) = the angle of the front wheels from -1 to 1, 


#### def `stop(self)`

Brings the car to a stop and points the front wheels forward


#### def `__update(self)`

Publishes the current drive message

### class Controller

Handles input from the controller and exposes constant input state
per frame

#### class Buttons(Enum)

The buttons on the controller

#### class Trigger

The triggers on the controller

#### class Joystick(Enum)

The joysticks on the controller

#### def `__init__`

#### def `get_joystick(self, joystick)`

Returns the position of a certain joystick

Inputs:
joystick (Joystick enum) = which joystick to check

Output (float, float): The x and y coordinate of the joystick, with
each axis ranging from -1.0 to 1.0

#### def `get_trigger(self, trigger)`

Returns the position of a certain trigger

Inputs:
trigger (Trigger enum) = which trigger to check

Output (float): A value from 0.0 (not pressed) to 
1.0 (fully pressed)

#### def `is_down(self, button)`

Returns whether a certain button is currently pressed

Inputs:
button (Button enum) = which button to check

Output (bool): True if button is currently pressed

#### def `was_pressed(self, button)`

Returns whether a certain button was pressed this frame

Inputs:
button (Button enum) = which button to check

Output (bool): True if button is currently pressed and was not pressed
last frame

#### def `__controller_callback(self, msg)`

TODO: Docstring

#### def `def __convert_trigger_value(self, value)`

TODO: docstring

#### def `__update`

Updates the stored input registers when the current frame ends

### class Camera

Provides the user with basic image capture functionality so they can get rbg/depth information from the Camera on the robot.

#### def `__init__(self)`

#### def `__del__(self)`

#### def `get_image(self)`

Get's color image data from Intel Realsense Camera and returns the raw data in numpy array

#### def `get_depth_image(self)`

Get's color+depth image (4 channels) from Intel Realsense Camera and returns the raw data in numpy array

