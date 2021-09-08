# General Defines used in VESC and PCAN

######################### Common Defines #########################
msg_type = {'0x0':'PCAN_MESSAGE_STANDARD',
            '0x00':'PCAN_MESSAGE_STANDARD', 
            '0x1':'PCAN_MESSAGE_RTR',
            '0x01':'PCAN_MESSAGE_RTR', 
            '0x2':'PCAN_MESSAGE_EXTENDED',
            '0x02':'PCAN_MESSAGE_EXTENDED', 
            '0x4':'PCAN_MESSAGE_FD',
            '0x04':'PCAN_MESSAGE_FD',
            '0x8':'PCAN_MESSAGE_BRS',
            '0x08':'PCAN_MESSAGE_BRS',
            '0x10':'PCAN_MESSAGE_ESI',
            '0x40':'PCAN_MESSAGE_ERRFRAME',
            '0x80':'PCAN_MESSAGE_STATUS' 
            }

COMM_PACKET_ID = {
'COMM_FW_VERSION':0,
'COMM_JUMP_TO_BOOTLOADER':1,
'COMM_ERASE_NEW_APP':2,
'COMM_WRITE_NEW_APP_DATA':3,
'COMM_GET_VALUES':4,
'COMM_SET_DUTY':5,
'COMM_SET_CURRENT':6,
'COMM_SET_CURRENT_BRAKE':7,
'COMM_SET_RPM':8,
'COMM_SET_POS':9,
'COMM_SET_HANDBRAKE':10,
'COMM_SET_DETECT':11,
'COMM_SET_SERVO_POS':12,
'COMM_SET_MCCONF':13,
'COMM_GET_MCCONF':14,
'COMM_GET_MCCONF_DEFAULT':15,
'COMM_SET_APPCONF':16,
'COMM_GET_APPCONF':17,
'COMM_GET_APPCONF_DEFAULT':18,
'COMM_SAMPLE_PRINT':19,
'COMM_TERMINAL_CMD':20,
'COMM_PRINT':21,
'COMM_ROTOR_POSITION':22,
'COMM_EXPERIMENT_SAMPLE':23,
'COMM_DETECT_MOTOR_PARAM':24,
'COMM_DETECT_MOTOR_R_L':25,
'COMM_DETECT_MOTOR_FLUX_LINKAGE':26,
'COMM_DETECT_ENCODER':27,
'COMM_DETECT_HALL_FOC':28,
'COMM_REBOOT':29,
'COMM_ALIVE':30,
'COMM_FORWARD_CAN':34,
'COMM_CUSTOM_APP_DATA':36,
'COMM_PING_CAN':62,
'COMM_GET_IMU_DATA':65,
}

CAN_PACKET_ID = {
'CAN_PACKET_SET_DUTY':0,
'CAN_PACKET_SET_CURRENT':1,
'CAN_PACKET_SET_CURRENT_BRAKE':2,
'CAN_PACKET_SET_RPM':3,
'CAN_PACKET_SET_POS':4,
'CAN_PACKET_FILL_RX_BUFFER':5,
'CAN_PACKET_FILL_RX_BUFFER_LONG':6,
'CAN_PACKET_PROCESS_RX_BUFFER':7,
'CAN_PACKET_PROCESS_SHORT_BUFFER':8,
'CAN_PACKET_STATUS':9,
'CAN_PACKET_SET_CURRENT_REL':10,
'CAN_PACKET_SET_CURRENT_BRAKE_REL':11,
'CAN_PACKET_SET_CURRENT_HANDBRAKE':12,
'CAN_PACKET_SET_CURRENT_HANDBRAKE_REL':13,
'CAN_PACKET_STATUS_2':14,
'CAN_PACKET_STATUS_3':15,
'CAN_PACKET_STATUS_4':16,
'CAN_PACKET_PING':17,
'CAN_PACKET_PONG':18,
'CAN_PACKET_DETECT_APPLY_ALL_FOC':19,
'CAN_PACKET_DETECT_APPLY_ALL_FOC_RES':20,
'CAN_PACKET_CONF_CURRENT_LIMITS':21,
'CAN_PACKET_CONF_STORE_CURRENT_LIMITS':22,
'CAN_PACKET_CONF_CURRENT_LIMITS_IN':23,
'CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN':24,
'CAN_PACKET_CONF_FOC_ERPMS':25,
'CAN_PACKET_CONF_STORE_FOC_ERPMS':26,
'CAN_PACKET_STATUS_5':27,
'CAN_PACKET_POLL_TS5700N8501_STATUS':28,
'CAN_PACKET_CONF_BATTERY_CUT':29,
'CAN_PACKET_CONF_STORE_BATTERY_CUT':30,
'CAN_PACKET_SHUTDOWN':31,
}

OPENROBOT_HOST_TYPE = {
'UNKNOWN':0,
'ARDUINO_MEGA':1,
'ARDUINO_DUE':2,
'ARDUINO_TEENSY_32':3,
'ARDUINO_TEENSY_36':4,
'USB':5,
'CAN_DIRECT_MSG':6
}

COMM_PACKET_ID_OPENROBOT = {
'COMM_SET_RELEASE':100,
'COMM_SET_DPS':101,
'COMM_SET_DPS_VMAX':102,
'COMM_SET_DPS_AMAX':103,
'COMM_SET_SERVO':104,
'COMM_SET_TRAJ':105
}

######################### Common Functions ######################### 
def list2hex(input):
    list2hex = ' '.join('0x{:02x}'.format(input[i]) for i in range(len(input)))
    return list2hex

def list2hex_nospace(input):
    list2hex = ' '.join('{:02x}'.format(input[i]) for i in range(len(input)))
    return list2hex

def list2chr(input):
    list2chr = ''.join(chr(input[i]) for i in range(len(input)))
    return list2chr

def status_negative_value_process_2_byte(raw_value):
    # Negative hex value process
    if raw_value & 0x8000: # MSB set -> neg.
        value = -((~raw_value & 0xffff) + 1)
    else:
        value = raw_value
    return value

def status_negative_value_process_3_byte(raw_value):
    # Negative hex value process
    if raw_value & 0x800000: # MSB set -> neg.
        value = -((~raw_value & 0xffffff) + 1)
    else:
        value = raw_value
    return value

def status_negative_value_process_4_byte(raw_value):
    # Negative hex value process
    if raw_value & 0x80000000: # MSB set -> neg.
        value = -((~raw_value & 0xffffffff) + 1)
    else:
        value = raw_value
    return value