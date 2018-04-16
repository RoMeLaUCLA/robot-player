# Robot Player

The Robot Player package is a small, self contained package that aims to simplify communicating with simulators and robots. Often it's required to have little 'toggle' switches everywhere in your code to have your programs communicate with a simulator and the hardware. This package aims to simplify that by presenting a uniform interface through the MotionManager class.

Each 'player' is a class designed to abstract the boilerplate and small details like port numbers and handles away from the user.

** Currently, all that is needed is to build the C libraries **


## Installation instructions

### Dependencies
Currently robot-player is being tested with python 2.7. It might work with python 3 as well, but I haven't tested it.

This tutorial assumes that you are using a python virtual environment. If you've installed virtualenv and virtualenvwrapper, you can set things up very easily with the following lines of code:
```
mkvirtualenv -p /usr/bin/python2.7 --no-site-packages player2
```

You'll also need numpy.
```python
pip install numpy
```

```bash
git clone https://github.com/RoMeLaUCLA/robot-player.git
cd robot-player
git checkout release
pip install .
```

That's it!

## Tutorial
Full examples of how to use the MotionManager class with the DxlInterface and VrepInterface will be provided in the tests directory.

Here's an annotated example from test_double_joint.py:
```python
from robot_player import MotionManager, VrepOptions
import numpy as np
from time import sleep

motor_id = [1,2]
dt = .01

with MotionManager(motor_id, dt, VrepOptions(joint_prefix="joint")) as mm:

    for i in range(100):
        assert(mm.get_current_position([1,2]) == mm.get_all_current_position())
        # send command to just one motor
        mm.set_command_position([1], [1])
        mm.set_command_position([2], [1])
        mm.advance_timestep()
    # check that motors got to right place
    pos = mm.get_all_current_position()
    assert(np.allclose(pos,[1,1]))

    for i in range(100):
        mm.set_all_command_position([-1,-2])
        mm.advance_timestep()
    # check that motors got to right place
    pos = mm.get_all_current_position()
    assert(np.allclose(pos,[-1,-2]))

    # joint velocity
    print mm.get_joint_velocity([1,2])
    assert(mm.get_joint_velocity([1,2]) == mm.get_all_joint_velocity())

    # switch to force controlled tests
    mm.set_joint_ctrl_loop([1,2],[False, True])
    assert(mm.get_joint_ctrl_loop([1,2])== [0,1])



    mm.set_joint_ctrl_loop([1, 2], [False, False])
    print("set joint velocity")
    for i in range(100):
        mm.set_joint_velocity([1,2], [1,-1])
        mm.advance_timestep()
    print("set all joint velocity")
    for i in range(100):
        mm.set_all_joint_velocity([1,2], [2,-2])
        mm.advance_timestep()

    print("reset position")
    mm.set_joint_ctrl_loop([1, 2], [True, True])
    mm.set_all_joint_effort([100, 100], send=False)
    for i in range(1000):
        mm.set_all_command_position([0,0])
        mm.advance_timestep()

    mm.set_joint_ctrl_loop([1, 2], [False, False])
    print("test set_joint_effort")
    for i in range(100):
        # get joint effort is also run in the set_all_joint_effort part
        mm.set_all_joint_effort([.25,0])
        mm.advance_timestep()
    for i in range(100):
        mm.set_joint_effort([1,2], [0,0])
        mm.advance_timestep()
```

We view robots as a collection of joints, with "devices" that allow us to command the joints and make them do various actions. These devices can be a simulator (eg. VREP) or a motor controller (USB2Dynamixel).

MotionManagers abstract the process of managing what could be a rapidly changing interface for the underlying devices by providing a clean, uniform interface. You just need to instantiate a device options class and pass it as an argument to the MotionManager. In the snippet below, we've done just that:

```python
from robot_player import MotionManager, VrepOptions

...

vi = VrepOptions(joint_prefix="joint")
mm = MotionManager(motor_id, dt, vi)
mm.initialize() # calls startup routines for all devices

```

For quick setup, this is fine. However, most devices tend to have some setting up at the beginning and cleaning up to do once we're finished using them, eg. stopping the simulation or closing the port. We can make sure to do that even if there's an exception or we stop the program in the middle of debugging by using the a context manager and the with keyword:

```python
from robot_player import MotionManager, VrepOptions
import numpy as np
from time import sleep

motor_id = [1,2]
dt = .01

with MotionManager(motor_id, dt, VrepOptions(joint_prefix="joint")) as mm:

    # send commands to the device
```

You can look up all of the options for the VrepOptions and DxlOptions in their respective files. When possible, they are keyword arguments with default values so that you don't have to remember everything. When using MotionManager and the `with` keyword, `initialize()` is automatically called for you.

 ### Joint Commands

 All joint commands have a specific structure:

 ```python
 mm.get_current_position(ids)
 mm.set_command_position(ids, commands)
```

`ids` is a list or tuple of values that has the joint ids to send commands to. `commands` is a list or tuple that has the joint commands to send to the motors. The units for the commands are radians, rad/s or N-m for angular rotating joints, and meters, m/s or N for prismatic joints. Unit conversion to the appropriate units for the device is automatically handled by the player interfaces.

In addition to the per-id joint commands, there is also an "all" command that lets you leave out the ids command since we usually just command all of the joints anyways:

```python
mm.get_all_current_position()
mm.set_all_command_position(commands)
```

In general, all of the getters have the syntax

```python
get_<name of parameter>(list_of_motor_ids)
get_all_<name of parameter>()
```
and all of the setters have the syntax
```python
set_<name of parameter>(list_of_motor_ids, list_of_commands)
set_all_<name of parameter>(commands)
```

### Player offsets
Occasionally, different players will have different joint offsets or the axes will be flipped. This might occur when it is conceptually easier to reason about joint angles being the same for two pairs of mirrored limbs, or when physical constraints require actuators to be flipped around.

Since this is usually a fairly last-minute calculation that may or may not be needed, we avoid the overhead of doing this on every call and simply provide some helper functions to do this: `to_player angle_offset` and `from_player_angle_offset`.

The motion manager does not do any angle conversion by default, so handling of the input and output data is completely handled by the individual player interfaces.

However, frequently different players will have different angles and offsets so the functions `to_player_angle_offset()` and `from_player_offset()` are provided to make this easier on the user.

To use them, define a class (anything will do, but I usually call it <name of player>AngleOffset)

```python
class VrepAngleOffset:
    """
    angle_trans is a list of 1 or -1 depending on if the joint axis is flipped or not
    angle_offset is a list of angle offsets in radians
    """
    angle_trans = [1, -1, 1, -1]
    angle_offset = [-math.pi/2, -math.pi/2, 0, 0]
```

We can see from this example that the Nabi_s robot needs motor axes 2 and 4 to have their joint axes flipped and that there is an angle offset of -pi/2 for the first two motors, which correspond to the hips. This greatly simplifies writing the kinematics for the robot.

After defining the AngleOffset class, import it and use it to convert angles.

```python
player_offset = VrepAngleOffset
qcurr = from_player_angle_offset(mm.get_all_current_position(), player_offset)

# calculate desired joint angles...

q = to_player_angle_offset(qdes,player_offset)
mm.set_all_command_position(q)
```

## DxlInterface

The DxlInterface is a class to talk to multiple dynamixel chains, abstracted as DxlPorts. What follows is a more in-depth look at the way this device is implemented, for those looking to extend it.


### Initialization
Specify the options for each usb port like this:
```python
device_opts = DxlOptions(motor_ids =[(1,2,3,4,5,6), (7,8,9,10,11,12),(19,20)],
                         motor_types=['DXLPRO','DXLPRO','MX106')],
                         ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"],
                         baudrate=3000000,
                         protocol_version=2
                         )
```

This device automatically multiplexes calls to the individual ports to make it seem like multiple ports are one contiguous device.

When you write to the motors, they will be written all at once, using the syncwrite or syncread commands.
The order of the motors is implicit- the order that you initialized the motors in.
First by device, then by the order that you listed the motors.


```python
# device1 ids: [1,2,3,4,5]
# device2 ids: [6,7,8,9,10]
DI.set_all_command_position([1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5])
```

The above ordering corresponds to the id order 1,2,3,4,5,6,7,8,9,10 and writes the value 1.5 radians to all of the motors.

### Dynamixel Control Table
Each Dynamixel motor has a different control table based on which protocol (1.0 or 2.0) and which motor model (MX-28, DXLPRO, MX-106) you are using. To keep users from having to deal with this annoying difference between all of the motors, the motor model of each motor is determined at startup and an appropriate control table is selected for each motor.

The file that holds all of these mappings is named dxl\_control\_table.py and is (currently as of 1-13-18) located in rf/player/dynamixel_sdk/dxl_control_table.

Each control table class is implemented as a class. Here's an annotated snippet of the DXLPRO 2.0 class:

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
    # Model numbers can be found on the Robotis website. They are accessible using the dynamixel.pingGetModelNum method.
    H54_200_S500_R = 54024
    H54_200_B500_R = 54152
    ...
    L54_30_S400_R = 37928
    # L42 Dynamixel has a different control table

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

    LEN_GOAL_POSITION = 4
    LEN_PRESENT_POSITION = 4

    # =====================================================================
    # EEPROM
    # =====================================================================
    MODEL_NUMBER = 0
    MODEL_INFORMATION = 2
    FIRMWARE_VERSION = 6
    MOTOR_ID = 7
    BAUD_RATE = 8
    RETURN_DELAY_TIME = 9
    OPERATING_MODE = 11
    HOMING_OFFSET = 13
    MOVING_THRESHOLD = 17
    TEMPERATURE_LIMIT = 21
    MAX_VOLTAGE_LIMIT = 22
    MIN_VOLTAGE_LIMIT = 24
    ACCELERATION_LIMIT = 26
    TORQUE_LIMIT = 30
    VELOCITY_LIMIT = 32
    MAX_POSITION_LIMIT = 36
    MIN_POSITION_LIMIT = 40
    EXTERNAL_PORT_MODE_1 = 44
    EXTERNAL_PORT_MODE_2 = 45
    EXTERNAL_PORT_MODE_3 = 46
    EXTERNAL_PORT_MODE_4 = 47
    SHUTDOWN = 48
    INDIRECT_ADDRESS = 49


    """
    can do other indirect addresses by adding INDIRECT_ADDRESS + 2*(address_num-1)
    eg. INDIRECT_ADDRESS_5 = INDIRECT_ADDRESS + 2*(5-1) = 57

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
    FEEDFORWARD_2ND_GAIN = 88
    FEEDFORWARD_1ST_GAIN = 90
    GOAL_POSITION = 596
    GOAL_VELOCITY = 600
    GOAL_TORQUE = 604
    GOAL_ACCELERATION = 606
    MOVING = 610
    PRESENT_POSITION = 611
    PRESENT_VELOCITY = 615
    PRESENT_CURRENT = 621
    PRESENT_INPUT_VOLTAGE = 623
    PRESENT_TEMPERATURE = 625
    EXTERNAL_PORT_DATA_1 = 626
    EXTERNAL_PORT_DATA_2 = 628
    EXTERNAL_PORT_DATA_3 = 630
    EXTERNAL_PORT_DATA_4 = 632
    INDIRECT_DATA = 634

    """
    can do other indirect data by adding INDIRECT_DATA + 2*(address_num-1)
    eg. INDIRECT_DATA_5 = INDIRECT_DATA + 2*(5-1) = 57

    up to INDIRECT_DATA_256 (889)

    """

    REGISTERED_INSTRUCTION = 890
    STATUS_RETURN_LEVEL = 891
    HARDWARE_ERROR_STATUS = 892
```

Here's some selected elements of the code:

#### Model numbers
Model numbers can be found on the Robotis website. They are accessible using the dynamixel.pingGetModelNum method.
```
    H54_200_S500_R = 54024
    H54_200_B500_R = 54152
    ...
    L54_30_S400_R = 37928
```

#### Resolution

All motors are commanded with radians and converted to dynamixel counts before writing commands.

```python
    resolution = {H54_200_S500_R: 501922,

                  H54_100_S500_R: 501922,
                  ...
                  L54_30_S500_R: 180692*2,
                  }
```

#### Byte length of important control table values

Certain control table values are accessed often with the sync write command, but they have differing byte lengths per motor model. Each control table class that wants to use sync write/sync read needs to have these.

```python
    LEN_GOAL_POSITION = 4
    LEN_PRESENT_POSITION = 4
```

The rest of the class is just variables with a value assigned to them. Some of the older DXL models using protocol 1.0 had their values listed in hexadecimal but it's fine to just use decimal equivalents.

```python
    # =====================================================================
    # EEPROM
    # =====================================================================
    MODEL_NUMBER = 0
    MODEL_INFORMATION = 2 # goes to 2 next because Model number has byte length 2
    FIRMWARE_VERSION = 6
    MOTOR_ID = 7
    BAUD_RATE = 8
   ....
    SHUTDOWN = 48
    INDIRECT_ADDRESS = 49


    """
    can do other indirect addresses by adding INDIRECT_ADDRESS + 2*(address_num-1)
    eg. INDIRECT_ADDRESS_5 = INDIRECT_ADDRESS + 2*(5-1) = 57

    up to INDIRECT_ADDRESS_256 (569)


    """
```

There's difference in the code between the EEPROM and RAM functions, but the EEPROM values can only be written to when the motor torque is off. The RAM values can only be written to when the motor torque is on.

```python

    # =====================================================================
    # RAM
    # =====================================================================
    TORQUE_ENABLE = 562
    LED_RED = 563
    LED_GREEN = 564
    ...
    EXTERNAL_PORT_DATA_4 = 632
    INDIRECT_DATA = 634

    """
    can do other indirect data by adding INDIRECT_DATA + 2*(address_num-1)
    eg. INDIRECT_DATA_5 = INDIRECT_DATA + 2*(5-1) = 57

    up to INDIRECT_DATA_256 (889)

    """

    REGISTERED_INSTRUCTION = 890
    STATUS_RETURN_LEVEL = 891
    HARDWARE_ERROR_STATUS = 892
```
