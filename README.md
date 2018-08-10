# Robot Player

The Robot Player package is a small, self contained package that aims to simplify communicating with simulators and robots. Often it's required to have little 'toggle' switches everywhere in your code to have your programs communicate with a simulator and the hardware. This package aims to simplify that by presenting a uniform interface through the MotionManager class.

Each 'player' is a class designed to abstract the boilerplate and small details like port numbers and handles away from the user.

## Installation instructions

### C Libraries
#### Windows
Open the `dxl_x[86|64].sln` file in the dxl/c/build/win\[32|64] directory and build using Visual Studio.

#### Mac and Linux
Navigate to the dxl/c/build directory and run cmake in the correct directory for your system.

### Dependencies
Currently robot-player is being developed with python 2.7. It is not completely supported in python 3 because of outdated code in the Dynamixel SDK, however the VREP functions will work fine.

This tutorial assumes that you are using a python virtual environment. If you've installed virtualenv and virtualenvwrapper, you can set things up very easily with the following lines of code:
```
mkvirtualenv -p /usr/bin/python2.7 --no-site-packages robot-player
```

Once your virtual environment is created and activated, clone and install the repository.
```bash
git clone https://github.com/RoMeLaUCLA/robot-player.git
pip install -e robot-player
```
You'll also need numpy.
```
pip install numpy
```

That's it!

## Tutorial
Examples of how to use the MotionManager class with the DxlInterface and VrepInterface are provided in the tests directory. The VREP tests also include scene files in the tests/vrep directory.

### Setup
We view robots as a collection of joints, with "devices" that allow us to command the joints and make them do various actions. These devices can be a simulator (e.g. VREP) or a motor controller (e.g. USB2Dynamixel).

MotionManagers abstract the process of managing what could be a rapidly changing interface for the underlying devices by providing a clean, uniform interface. You just need to instantiate a device options class and pass it as an argument to the MotionManager. In the snippet below, we've done just that:

```python
from robot_player import MotionManager, VrepOptions

# ...

vi = VrepOptions(joint_prefix="joint")
mm = MotionManager(motor_id, dt, vi)
mm.initialize() # calls startup routines for all devices
```

For quick setup, this is fine. However, most devices tend to have some setting up at the beginning and cleaning up to do once we're finished using them, e.g. stopping the simulation or closing the port. We can make sure to do that even if there's an exception or we stop the program in the middle of debugging by using a context manager and the `with` keyword:

```python
from robot_player import MotionManager, VrepOptions

# ...

with MotionManager(motor_id, dt, VrepOptions(joint_prefix="joint")) as mm:
    # send commands to the device
```

You can look up all of the options for the VrepOptions and DxlOptions in their respective files. When possible, they are keyword arguments with default values so that you don't have to remember everything. When using MotionManager and the `with` keyword, `initialize()` is automatically called for you.

### Joint Commands

All joint commands have a specific structure:

 ```python
mm.get_present_position(ids)
mm.set_goal_position(ids, commands)
```

`ids` is a list or tuple of values that has the joint ids to send commands to. `commands` is a list or tuple that has the joint commands to send to the motors. The units for the commands are shown in the table below. Unit conversion to the appropriate units for the device is automatically handled by the player interfaces.

command type | rotating joint | prismatic joint
-------------|----------------|----------------
position     | rad            | m
velocity     | rad/s          | m/s
effort       | N-m            | N

In addition to the per-id joint commands, there is also an "all" command that lets you leave out the ids command since we usually just command all of the joints anyways

```python
mm.get_all_present_position()
mm.set_all_goal_position(commands)
```

In general, all of the getters have the syntax

```python
get_<name of parameter>(motor_ids)
get_all_<name of parameter>()
```
and all of the setters have the syntax
```python
set_<name of parameter>(motor_ids, commands)
set_all_<name of parameter>(commands)
```

### Player offsets
Occasionally, different players will have different joint offsets or the axes will be flipped. This might occur when it is conceptually easier to reason about joint angles being the same for two pairs of mirrored limbs, or when physical constraints require actuators to be flipped around.

The motion manager does not do any angle conversion by default, so handling of the input and output data is completely handled by the individual player interfaces. Since this is usually a fairly last-minute calculation that may or may not be needed, we avoid the overhead of doing this on every call and simply provide some helper functions to do this: `to_player_angle_offset` and `from_player_angle_offset`.

To use them, make an instance of the AngleOffset class

```python
nabi_offset = AngleOffset(trans=[1, -1, 1, -1], offset=[pi/2, pi/2, 0, 0])
```

We can see from this example that the Nabi robot needs motor axes 2 and 4 to have their joint axes flipped and that there is an angle offset of -pi/2 for the first two motors, which correspond to the hips. This greatly simplifies writing the kinematics for the robot.

Once created, the `AngleOffset` instance can be used with the player angle offset functions

```python
qcurr = from_player_angle_offset(mm.get_all_present_position(), nabi_offset)

# calculate desired q for all joints...

mm.set_all_present_position(to_player_angle_offset(qdes, nabi_offset)
```

### VrepInterface

#### Streaming initialization
All of the setter commands have the opmode simx_opmode_oneshot, which sends a single command without waiting for a reply.
We use this instead of using blocking commands because they make the simulation run much faster.

Some of the getter commands might have the option to use either the opmode simx_opmode_streaming or simx_opmode_buffer. 
Calling the getter commands once at the beginning of your script with the keyword argument streaming=True sets VREP to automatically prepare the data and put it into a queue for all future calls.
This dramatically increases the simulation speed.

### DxlInterface

The DxlInterface is a class to talk to multiple dynamixel chains, abstracted as DxlPorts. What follows is a more in-depth look at the way this device is implemented for those looking to extend it.

#### Initialization
Specify the options for each usb port like this:
```python
device_opts = DxlOptions(motor_ids =[(1, 2, 3, 4, 5, 6), (7, 8, 9, 10, 11, 12), (19, 20)],
                         motor_types=['DXLPRO','DXLPRO','MX106'],
                         ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"],
                         baudrate=3000000,
                         protocol_version=2)
```
This specifies that there are three chains, the first one existing on port USB0 and having 6 DXLPROs. All motors on the same chain must be of the same type.

This device automatically multiplexes calls to the individual ports to make it seem like multiple ports are one contiguous device.

When you write to the motors, they will be written all at once, using the `_sync_write` or `_sync_read` commands. The order of the commands must match the order in which the motors were initialized in `DxlOptions` above.

```python
# device1 ids: [1, 2, 3, 4, 5]
# device2 ids: [6, 7, 8, 9, 10]
DI.set_all_goal_position([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5])
```

The above ordering corresponds to the id order 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 and writes the value 1.5 radians to all the motors.

#### Dynamixel Control Table
Each Dynamixel motor has a different control table based on which protocol (i.e. 1.0 or 2.0) and which motor model (e.g. DXLPRO H54_200_S500_R) you are using. To keep users from having to deal with this annoying difference between all of the motors, the model of each motor is determined at startup and an appropriate control table is selected for each motor.

Each control table is implemented as a class in [dxl_control_table.py](robot_player/dxl/dxl_control_table.py) Here's an annotated snippet of the DXLPRO 2.0 class:

```python

class DXLPRO:
    """
    DXL Control Table, Protocol 2.0
    =================
    Relevant register values from the online manual.
    Reference:
    http://support.robotis.com/en/product/actuator/dynamixel_pro.htm
    http://support.robotis.com/en/product/actuator/dynamixel_pro/dynamixelpro/control_table.htm

    """
    # Model numbers can be found on the Robotis website.
    H54_200_S500_R = 54024
    H54_200_B500_R = 54152
    ...
    L54_30_S400_R = 37928
    # L42 Dynamixel has a different control table

    # conversion factors dynamixel counts to radians
    resolution = {H54_200_S500_R: 501922,
                  H54_100_S500_R: 501922,
                  H54_200_B500_R: 501922,
                  H42_20_S300_R: 303750,
                  M54_60_S250_R: 125708*2,
                  M54_40_S250_R: 131593*2,
                  M42_10_S260_R: 131593*2,
                  L54_50_S290_R: 103846*2,
                  L54_50_S500_R: 180692*2,
                  L54_30_S400_R: 144197*2,
                  L54_30_S500_R: 180692*2,
                  }

    # conversion factors for velocity units to RPM
    VEL_UNIT = {H54_200_S500_R: 0.00199234,
                H54_100_S500_R: 0.00199234,
                H54_200_B500_R: 0.00199234,
                H42_20_S300_R: 0.00329218,
                M54_60_S250_R: 0.00397746,
                M54_40_S250_R: 0.00397746,
                M42_10_S260_R: 0.00389076,
                L54_50_S290_R: 0.00346667,
                L54_50_S500_R: 0.00199234,
                L54_30_S400_R: 0.00249657,
                L54_30_S500_R: 0.00199234,
                }

    # read/write command lengths
    LEN_GOAL_POSITION = 4
    LEN_PRESENT_POSITION = 4
    LEN_GOAL_VELOCITY = 4
    LEN_PRESENT_VELOCITY = 4
    LEN_GOAL_EFFORT = 2
    LEN_PRESENT_EFFORT = 2

    # =====================================================================
    # EEPROM
    # =====================================================================
    MODEL_NUMBER = 0
    MODEL_INFORMATION = 2
    VERSION_OF_FIRMWARE = 6
    ID = 7
    BAUD_RATE = 8
    RETURN_DELAY_TIME = 9
    OPERATING_MODE = 11
    HOMING_OFFSET = 13
    MOVING_THRESHOLD = 17
    TEMPERATURE_LIMIT = 21
    MAX_VOLTAGE_LIMIT = 22
    MIN_VOLTAGE_LIMIT = 24
    ACCELERATION_LIMIT = 26
    EFFORT_LIMIT = 30
    VELOCITY_LIMIT = 32
    MAX_POSITION_LIMIT = 36
    MIN_POSITION_LIMIT = 40
    EXTERNAL_PORT_MODE_1 = 44
    EXTERNAL_PORT_MODE_2 = 45
    EXTERNAL_PORT_MODE_3 = 46
    EXTERNAL_PORT_MODE_4 = 47
    SHUTDOWN = 48
    INDIRECT_ADDRESS_1 = 49
    """
    can do other indirect addresses by adding INDIRECT_ADDRESS_1 + 2*(address_num-1)
    e.g. INDIRECT_ADDRESS_5 = INDIRECT_ADDRESS_1 + 2*(5-1) = 57

    up to INDIRECT_ADDRESS_256 (569)
    """

    # =====================================================================
    # RAM
    # =====================================================================
    TORQUE_ENABLE = 562
    LED_RED = 563
    LED_GREEN = 564
    LED_BLUE = 565
    VELOCITY_I_GAIN = 586
    VELOCITY_P_GAIN = 588
    POSITION_P_GAIN = 594
    GOAL_POSITION = 596
    GOAL_VELOCITY = 600
    GOAL_EFFORT = 604
    GOAL_ACCELERATION = 606
    MOVING = 610
    PRESENT_POSITION = 611
    PRESENT_VELOCITY = 615
    PRESENT_EFFORT = 621
    PRESENT_INPUT_VOLTAGE = 623
    PRESENT_TEMPERATURE = 625
    EXTERNAL_PORT_DATA_1 = 626
    EXTERNAL_PORT_DATA_2 = 628
    EXTERNAL_PORT_DATA_3 = 630
    EXTERNAL_PORT_DATA_4 = 632
    INDIRECT_DATA_1 = 634
    """
    can do other indirect data by adding INDIRECT_DATA_1 + 2*(address_num-1)
    e.g. INDIRECT_DATA_5 = INDIRECT_DATA_1 + 2*(5-1) = 57

    up to INDIRECT_DATA_256 (889)
    """
    REGISTERED_INSTRUCTION = 890
    STATUS_RETURN_LEVEL = 891
    HARDWARE_ERROR_STATUS = 892
```

Here's some selected elements of the code:

##### Model numbers
Model numbers can be found on the Robotis website. They are accessible using the dynamixel.pingGetModelNum method. Example model numbers for DXLPRO:
```
    H54_200_S500_R = 54024
    H54_200_B500_R = 54152
    ...
    L54_30_S400_R = 37928
```

##### Resolution
All motors are commanded with radians and automatically converted to dynamixel counts before writing commands. The conversion factor is based on the model number and comes from a shift table defined at the top of `dxl_control_table.py`.

```
H54_200_S500_R: 501922
H54_100_S500_R: 501922
...
L54_30_S500_R: 180692*2
```

##### Byte length of important control table values
Certain control table values are accessed often with the `_sync_write` command, but they have differing byte lengths depending on motor model. Each control table class that wants to use read/write commands needs to have these.

```
LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4
...
```

The rest of the class is just variables with a value assigned to them. Some of the older DXL models using protocol 1.0 had their values listed in hexadecimal but it's fine to just use decimal equivalents.

```python
# =====================================================================
# EEPROM
# =====================================================================
MODEL_NUMBER = 0
MODEL_INFORMATION = 2

# ...

"""
can do other indirect addresses by adding INDIRECT_ADDRESS_1 + 2*(address_num-1)
e.g. INDIRECT_ADDRESS_5 = INDIRECT_ADDRESS_1 + 2*(5-1) = 57

up to INDIRECT_ADDRESS_256 (569)
"""
```

There's difference in the code between the EEPROM and RAM functions, but the EEPROM values can only be written to when the motor torque is off, whereas the RAM values can only be written to when the motor torque is on.

```python
# =====================================================================
# RAM
# =====================================================================
TORQUE_ENABLE = 562
LED_RED = 563

# ...

"""
can do other indirect data by adding INDIRECT_DATA_1 + 2*(address_num-1)
e.g. INDIRECT_DATA_5 = INDIRECT_DATA_1 + 2*(5-1) = 57

up to INDIRECT_DATA_256 (889)
"""
REGISTERED_INSTRUCTION = 890
STATUS_RETURN_LEVEL = 891
HARDWARE_ERROR_STATUS = 892
```
