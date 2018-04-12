"""
Control Table
=================
"""

class Instruction:
    """
    Instruction Constants
    =================
    Relevant register values from the online manual.
    Reference:
    http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm
    """

    # =====================================================================
    # Packet Instruction
    # =====================================================================
    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    RESET = 0x06
    SYNC_WRITE = 0x83
    BULK_READ = 0x92

    # Dynamixel 2.0 Protocol ONLY
    REBOOT = 0x08
    STATUS_RETURN = 0x55
    SYNC_READ = 0x82
    BULK_WRITE = 0x93

    ID_BROADCAST = 0xFE
    ID_CM = 200

class CM730:
    """
    CM730/CM740 Control Table
    =================
    Relevant register values from the online manual.
    Reference:
    http://support.robotis.com/ko/product/darwin-op/references/reference/hardware_specifications/electronics/sub_controller_(cm-730).htm
    http://support.robotis.com/en/product/robotis-op2/sub_controller(cm-740).htm
    """

    CM_730 = 29440

    # =====================================================================
    # EEPROM
    # =====================================================================
    MODEL_NUMBER_L = 0X00
    MODEL_NUMBER_H = 0X01
    FIRMWARE_VERSION = 0X02
    ID = 0X03
    BAUD_RATE = 0X04
    RETURN_DELAY_TIME = 0X05
    STATUS_RETURN_LEVEL = 0X10

    # =====================================================================
    # RAM
    # =====================================================================
    DXL_POWER = 0X18
    LED_PANNEL = 0X19
    LED_5_L = 0X1A
    LED_5_H = 0X1B
    LED_6_L = 0X1C
    LED_6_H = 0X1D
    LED_HEAD_L = 0X1A
    LED_HEAD_H = 0X1B
    LED_EYE_L = 0X1C
    LED_EYE_H = 0X1D
    BUTTON = 0X1E
    GYRO_Z_L = 0X26
    GYRO_Z_H = 0X27
    GYRO_Y_L = 0X28
    GYRO_Y_H = 0X29
    GYRO_X_L = 0X2A
    GYRO_X_H = 0X2B
    ACC_X_L = 0X2C
    ACC_X_H = 0X2D
    ACC_Y_L = 0X2E
    ACC_Y_H = 0X2F
    ACC_Z_L = 0X30
    ACC_Z_H = 0X31
    PRESENT_VOLTAGE = 0X32
    MIC_1_L = 0X33
    MIC_1_H = 0X34
    LEFT_MIC_L = 0X33
    LEFT_MIC_H = 0X34
    ADC_2_L = 0X35
    ADC_2_H = 0X36
    ADC_3_L = 0X37
    ADC_3_H = 0X38
    ADC_4_L = 0X39
    ADC_4_H = 0X3A
    ADC_5_L = 0X3B
    ADC_5_H = 0X3C
    ADC_6_L = 0X3D
    ADC_6_H = 0X3E
    ADC_7_L = 0X3F
    ADC_7_H = 0X40
    ADC_8_L = 0X41
    ADC_8_H = 0X42
    MIC_2_L = 0X43
    MIC_2_H = 0X44
    RIGHT_MIC_L = 0X43
    RIGHT_MIC_H = 0X44
    ADC_10_L = 0X45
    ADC_10_H = 0X46
    ADC_11_L = 0X47
    ADC_11_H = 0X48
    ADC_12_L = 0X49
    ADC_12_H = 0X4A
    ADC_13_L = 0X4B
    ADC_13_H = 0X4C
    ADC_14_L = 0X4D
    ADC_14_H = 0X4E
    ADC_15_L = 0X4F
    ADC_15_H = 0X50

    ID_CM = 200

    """
    INPUT_VOLTAGE   = 1,
    ANGLE_LIMIT     = 2,
    OVERHEATING     = 4,
    RANGE           = 8,
    CHECKSUM        = 16,
    OVERLOAD        = 32,
    INSTRUCTION     = 64,
    SUCCESS = 0,
    TX_CORRUPT = 1,
    TX_FAIL = 2,
    RX_FAIL = 3,
    RX_TIMEOUT = 4,
    RX_CORRUPT = 5,
    ID = 2
    LENGTH = 3
    INSTRUCTION = 4
    ERRBIT = 4
    PARAMETER = 5
    DEFAULT_BAUDNUMBER = 1
    """

    # =====================================================================
    # Packet Instruction
    # =====================================================================
    INST_PING = 0X01
    INST_READ = 0X02
    INST_WRITE = 0X03
    INST_REG_WRITE = 0X04
    INST_ACTION = 0X05
    INST_RESET = 0X06
    INST_SYNC_WRITE = 0x83
    INST_BULK_READ = 0x92


class MX106:
    """
    Dynamixel MX-106 Control Table, Protocol 2.0
    =================
    Relevant register values from the online manual.
    Reference:
    http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-106(2.0).htm
    """

    MX_106 = 321
    resolution = 4096

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
    DRIVE_MODE = 10
    OPERATING_MODE = 11
    SECONDARY_ID = 12
    PROTOCOL_VERSION = 13
    HOMING_OFFSET = 20
    MOVING_THRESHOLD = 24
    TEMPERATURE_LIMIT = 31
    MAX_VOLTAGE_LIMIT = 32
    MIN_VOLTAGE_LIMIT = 34
    PWM_LIMIT = 36
    CURRENT_LIMIT = 38
    ACCELERATION_LIMIT = 40
    VELOCITY_LIMIT = 44
    MAX_POSITION_LIMIT = 48
    MIN_POSITION_LIMIT = 52
    SHUTDOWN = 63

    # =====================================================================
    # RAM
    # =====================================================================
    TORQUE_ENABLE = 64
    LED = 65
    STATUS_RETURN_LEVEL = 68
    REGISTERED_INSTRUCTION = 69
    HARDWARE_ERROR_STATUS = 70
    VELOCITY_I_GAIN = 76
    VELOCITY_P_GAIN = 78
    POSITION_D_GAIN = 80
    POSITION_I_GAIN = 82
    POSITION_P_GAIN = 84
    FEEDFORWARD_2ND_GAIN = 88
    FEEDFORWARD_1ST_GAIN = 90
    BUS_WATCHDOG = 98
    GOAL_PWM = 100
    GOAL_CURRENT = 102
    GOAL_VELOCITY = 104
    PROFILE_ACCELERATION = 108
    PROFILE_VELOCITY = 112
    GOAL_POSITION = 116
    REALTIME_TICK = 120
    MOVING = 122
    MOVING_STATUS = 123
    PRESENT_PWM = 124
    PRESENT_CURRENT = 126
    PRESENT_VELOCITY = 128
    PRESENT_POSITION = 132
    VELOCITY_TRAJECTORY = 136
    POSITION_TRAJECTORY = 140
    PRESENT_INPUT_VOLTAGE = 144
    PRESENT_TEMPERATURE = 146



class MX106_P1:
    # only implemented limited functionality since Protocol 1.0 has different names

    MX_106_P1 = 320
    resolution = 4096

    LEN_GOAL_POSITION = 2
    LEN_PRESENT_POSITION = 2

    # =====================================================================
    # EEPROM
    # =====================================================================

    MODEL_NUMBER = 0x00
    FIRMWARE_VERSION = 0x02
    MOTOR_ID = 0x03
    BAUD_RATE = 0x04
    RETURN_DELAY_TIME = 0x05
    CW_ANGLE_LIMIT_L = 0x06
    CW_ANGLE_LIMIT_H = 0x07
    CCW_ANGLE_LIMIT_L = 0x08
    CCW_ANGLE_LIMIT_H = 0x09
    DXL_DRIVE_MODE = 0x0A
    DXL_HIGH_LIMIT_TEMP = 0x0B
    DXL_LOW_LIMIT_VOLTAGE = 0x0C
    DXL_HIGH_LIMIT_VOLTAGE = 0x0D
    DXL_MAX_TORQUE_L = 0x0E
    DXL_MAX_TORQUE_H = 0x0F
    DXL_STATUS_RETURN_LEVEL = 0x10
    DXL_ALARM_LED = 0x11
    DXL_ALARM_SHUTDOWN = 0x12
    DXL_MULTI_TURN_OFFSET_L = 0x14
    DXL_MULTI_TURN_OFFSET_H = 0x15
    DXL_RESOLUTION_DIVIDER = 0x16

    # =====================================================================
    # RAM
    # =====================================================================

    TORQUE_ENABLE = 0x18
    DXL_LED = 0x19
    DXL_D_GAIN = 0x1A
    DXL_I_GAIN = 0x1B
    DXL_P_GAIN = 0x1C
    GOAL_POSITION = 30
    DXL_MOVING_SPEED_L = 32
    DXL_MOVING_SPEED_H = 33
    DXL_TORQUE_LIMIT_L = 34
    DXL_TORQUE_LIMIT_H = 35
    PRESENT_POSITION = 36
    DXL_PRESENT_SPEED_L = 38
    DXL_PRESENT_SPEED_H = 39
    DXL_PRESENT_LOAD_L = 40
    DXL_PRESENT_LOAD_H = 41
    DXL_PRESENT_VOLTAGE = 42
    DXL_PRESENT_TEMPERATURE = 43
    DXL_REGISTERED = 44
    DXL_MOVING = 46
    DXL_LOCK = 47
    DXL_PUNCH_L = 48
    DXL_PUNCH_H = 49
    DXL_CURRENT_L = 68
    DXL_CURRENT_H = 69
    DXL_TORQUE_CONTROL_MODE_ENABLE = 70
    DXL_GOAL_TORQUE_L = 71
    DXL_GOAL_TORQUE_H = 72
    DXL_GOAL_ACCELERATION = 73


class MX28:
    """
    DXL Control Table
    =================
    Relevant register values from the online manual.
    Reference:
    http://support.robotis.com/ko/product/actuator/dynamixel/mx_series/mx-28.htm
    http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm
    """
    MX_28 = 30
    resolution = 4096

    # =====================================================================
    # EEPROM
    # =====================================================================
    MODEL_NUMBER_L = 0X00
    MODEL_NUMBER_H = 0X01
    FIRMWARE_VERSION = 0X02
    ID = 0X03
    BAUD_RATE = 0X04
    RETURN_DELAY_TIME = 0X05
    CW_ANGLE_LIMIT_L = 0X06
    CW_ANGLE_LIMIT_H = 0X07
    CCW_ANGLE_LIMIT_L = 0X08
    CCW_ANGLE_LIMIT_H = 0X09
    HIGH_LIMIT_TEMP = 0X0B
    LOW_LIMIT_VOLTAGE = 0X0C
    HIGH_LIMIT_VOLTAGE = 0X0D
    MAX_TORQUE_L = 0X0E
    MAX_TORQUE_H = 0X0F
    STATUS_RETURN_LEVEL = 0X10
    ALARM_LED = 0X11
    ALARM_SHUTDOWN = 0X12
    MULTI_TURN_OFFSET_L = 0X14
    MULTI_TURN_OFFSET_H = 0X15
    RESOLUTION_DIVIDER = 0X16

    # =====================================================================
    # RAM
    # =====================================================================
    TORQUE_ENABLE = 0X18
    LED = 0X19
    D_GAIN = 0X1A
    I_GAIN = 0X1B
    P_GAIN = 0X1C
    GOAL_POSITION_L = 0X1E
    GOAL_POSITION_H = 0X1F
    MOVING_SPEED_L = 0X20
    MOVING_SPEED_H = 0X21
    TORQUE_LIMIT_L = 0X22
    TORQUE_LIMIT_H = 0X23
    PRESENT_POSITION_L = 0X24
    PRESENT_POSITION_H = 0X25
    PRESENT_SPEED_L = 0X26
    PRESENT_SPEED_H = 0X27
    PRESENT_LOAD_L = 0X28
    PRESENT_LOAD_H = 0X29
    PRESENT_VOLTAGE = 0X2A
    PRESENT_TEMPERATURE = 0X2B
    REGISTERED = 0X2C
    MOVING = 0X2E
    LOCK = 0X2F
    PUNCH_L = 0X30
    PUNCH_H = 0X31
    GOAL_ACCELERATION = 0X49

    # Backward Compatibility, Compliance is replaced with PID
    CW_COMPLIANCE_MARGIN = 26   # DEPRECATED
    CCW_COMPLIANCE_MARGIN = 27  # DEPRECATED
    CW_COMPLIANCE_SLOPE = 28    # DEPRECATED
    CCW_COMPLIANCE_SLOPE = 29   # DEPRECATED

    """
    The relationship between PID and Compliance slope
     Slope  |   P Gain
    -------------------
      8     |    128
      16    |     64
      32    |     32
      64    |     16
      128   |      8
    The less the P gain, The larger the backlash, and the weaker the amount of output near goal position.
    At some extent, it is like a combined concept of margin and slope.
    It does not exactly match the concept of compliance previously used.
    So it is obvious if you see the difference in terms of motion.
    """


class AX12:

    AX_12 = 12

    # =====================================================================
    # EEPROM
    # =====================================================================
    MODEL_NUMBER_L = 0X00
    MODEL_NUMBER_H = 0X01
    FIRMWARE_VERSION = 0X02
    ID = 0X03
    BAUD_RATE = 0X04
    RETURN_DELAY_TIME = 0X05
    CW_ANGLE_LIMIT_L = 0X06
    CW_ANGLE_LIMIT_H = 0X07
    CCW_ANGLE_LIMIT_L = 0X08
    CCW_ANGLE_LIMIT_H = 0X09
    HIGH_LIMIT_TEMP = 0X0B
    LOW_LIMIT_VOLTAGE = 0X0C
    HIGH_LIMIT_VOLTAGE = 0X0D
    MAX_TORQUE_L = 0X0E
    MAX_TORQUE_H = 0X0F
    STATUS_RETURN_LEVEL = 0X10
    ALARM_LED = 0X11
    ALARM_SHUTDOWN = 0X12

    # =====================================================================
    # RAM
    # =====================================================================
    TORQUE_ENABLE = 0X18
    LED = 0X19
    CW_COMPLIANCE_MARGIN = 0X1A
    CCW_COMPLIANCE_MARGIN = 0X1B
    CW_COMPLIANCE_SLOPE = 0X1C
    CCW_COMPLIANCE_SLOPE = 0X1D
    GOAL_POSITION_L = 0X1E
    GOAL_POSITION_H = 0X1F
    MOVING_SPEED_L = 0X20
    MOVING_SPEED_H = 0X21
    TORQUE_LIMIT_L = 0X22
    TORQUE_LIMIT_H = 0X23
    PRESENT_POSITION_L = 0X24
    PRESENT_POSITION_H = 0X25
    PRESENT_SPEED_L = 0X26
    PRESENT_SPEED_H = 0X27
    PRESENT_LOAD_L = 0X28
    PRESENT_LOAD_H = 0X29
    PRESENT_VOLTAGE = 0X2A
    PRESENT_TEMPERATURE = 0X2B
    REGISTERED = 0X2C
    MOVING = 0X2E
    LOCK = 0X2F
    PUNCH_L = 0X30
    PUNCH_H = 0X31

class DXLPRO:
    """
    DXL Control Table, Protocol 2.0
    =================
    Relevant register values from the online manual.
    Reference:
    http://support.robotis.com/en/product/actuator/dynamixel_pro.htm
    http://support.robotis.com/en/product/actuator/dynamixel_pro/dynamixelpro/control_table.htm

    """

    # model numbers can be found on Robotis' website.
    H54_200_S500_R = 54024
    H54_200_B500_R = 54152
    H54_100_S500_R = 53768
    H42_20_S300_R = 51200
    M54_60_S250_R = 46352
    M54_40_S250_R =  46096
    M42_10_S260_R = 43288
    L54_50_S500_R = 38152
    L54_30_S500_R = 37896
    L54_50_S290_R = 38176
    L54_30_S400_R = 37928
    # L42 Dynamixel has a different control table

    # converts dynamixel counts to radians
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

    # byte length of certain control table values. TODO: update for control torques.
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


"""
Control Table for MX106
LEFT FOR BACKWARD COMPATIBILITY
"""

DXL_MODEL_NUMBER_L = 0x00
DXL_MODEL_NUMBER_H = 0x01
DXL_FIRMWARE_VERSION = 0x02
DXL_MOTOR_ID = 0x03
DXL_BAUD_RATE = 0x04
DXL_RETURN_DELAY_TIME = 0x05
DXL_CW_ANGLE_LIMIT_L = 0x06
DXL_CW_ANGLE_LIMIT_H = 0x07
DXL_CCW_ANGLE_LIMIT_L = 0x08
DXL_CCW_ANGLE_LIMIT_H = 0x09
DXL_DRIVE_MODE = 0x0A
DXL_HIGH_LIMIT_TEMP = 0x0B
DXL_LOW_LIMIT_VOLTAGE = 0x0C
DXL_HIGH_LIMIT_VOLTAGE = 0x0D
DXL_MAX_TORQUE_L = 0x0E
DXL_MAX_TORQUE_H = 0x0F
DXL_STATUS_RETURN_LEVEL = 0x10
DXL_ALARM_LED = 0x11
DXL_ALARM_SHUTDOWN = 0x12
DXL_MULTI_TURN_OFFSET_L = 0x14
DXL_MULTI_TURN_OFFSET_H = 0x15
DXL_RESOLUTION_DIVIDER = 0x16
DXL_TORQUE_ENABLE = 0x18
DXL_LED = 0x19
DXL_D_GAIN = 0x1A
DXL_I_GAIN = 0x1B
DXL_P_GAIN = 0x1C
DXL_GOAL_POSITION_L = 30
DXL_GOAL_POSITION_H = 31
DXL_MOVING_SPEED_L = 32
DXL_MOVING_SPEED_H = 33
DXL_TORQUE_LIMIT_L = 34
DXL_TORQUE_LIMIT_H = 35
DXL_PRESENT_POSITION_L = 36
DXL_PRESENT_POSITION_H = 37
DXL_PRESENT_SPEED_L = 38
DXL_PRESENT_SPEED_H = 39
DXL_PRESENT_LOAD_L = 40
DXL_PRESENT_LOAD_H = 41
DXL_PRESENT_VOLTAGE = 42
DXL_PRESENT_TEMPERATURE = 43
DXL_REGISTERED = 44
DXL_MOVING = 46
DXL_LOCK = 47
DXL_PUNCH_L = 48
DXL_PUNCH_H = 49
DXL_CURRENT_L = 68
DXL_CURRENT_H = 69
DXL_TORQUE_CONTROL_MODE_ENABLE = 70
DXL_GOAL_TORQUE_L = 71
DXL_GOAL_TORQUE_H = 72
DXL_GOAL_ACCELERATION = 73
DXL_PING = 0x01
DXL_READ_DATA = 0x02
DXL_WRITE_DATA = 0x03
DXL_REG_WRITE = 0x04
DXL_ACTION = 0x05
DXL_RESET = 0x06
DXL_SYNC_WRITE = 0x83
DXL_BULK_READ = 0x92
DXL_REBOOT = 0x08
DXL_STATUS_RETURN = 0x55
DXL_SYNC_READ = 0x82
DXL_BULK_WRITE = 0x92
DXL_BROADCAST_ID = 0xFE