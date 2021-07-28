import serial
import time
import signal
import threading
import sys
import glob
import ctypes
import serial.tools.list_ports
import platform
import vesc_crc as vc
from PCANBasic import *

######################### Defines #########################
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

######################### Common Variables #########################
debug_non_status_can_msg_print = False
debug_print_get_value_return = False
debug_print_custom_return = False

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

######################### PCAN #########################
class PCAN:
    def __init__(self):
        self.status1 = []

        self.exitThread_pcan = False   # 쓰레드 종료용 변수
        self.pcan_connection_flag = False

        self.time_prev = 0
        self.pc1 = PCANBasic()

    def get_status1(self):
        st_data = []
        for st in self.status1:
            # id, period_ms, pos, vel, curr, motor_temp
            temp = [st[0], "{:.1f}".format(st[2]), st[3], st[4], st[5], st[6]]
            st_data.append(temp)
        return st_data

    def pcan_Open(self, print_flag=True):
        result = self.pc1.Initialize(PCAN_USBBUS1, PCAN_BAUD_1M)
        if result != PCAN_ERROR_OK:
            # An error occurred, get a text describing the error and show it
            #
            result = self.pc1.GetErrorText(result)
            if print_flag: print(result[1])
        else:
            if print_flag: 
                print("PCAN-USB (Ch-1) was opened")
                self.pcan_connection_flag = True
        return result

    def pcan_Close(self, print_flag=True):
        # The USB Channel is released
        #
        result = self.pc1.Uninitialize(PCAN_USBBUS1)
        if result != PCAN_ERROR_OK:
            # An error occurred, get a text describing the error and show it
            #
            result = self.pc1.GetErrorText(result)
            if print_flag: print(result[1])
        else:
            if print_flag: 
                print("PCAN-USB (Ch-1) was closed")
                self.pcan_connection_flag = False

    def handler_pcan(self, signum, frame):
        self.exitThread_pcan = True

    def process_vesc_status_msg(self, msg):
        eid = msg[0]
        dlc = msg[1]
        timestamp = msg[2]
        data = msg[3]

        period_msec = 0

        #print(hex(eid))
    
        if len(data) == 8:
            temperature = (eid >> 16) & 0x1FFF
            motor_temp = status_negative_value_process_2_byte(temperature)/10.
            cmd = (eid >> 8) & 0xFF
            id = (eid) & 0xFF

            pos = 0x000000
            pos = pos | (data[0] << 16) 
            pos = pos | (data[1] << 8) 
            pos = pos | data[2]
            pos_rad = status_negative_value_process_3_byte(pos)/1000.
            
            vel = 0x000000
            vel = vel | (data[3] << 16)
            vel = vel | (data[4] << 8) 
            vel = vel | data[5]
            vel_rps = status_negative_value_process_3_byte(vel)/10000.
            
            curr = 0x0000
            curr = curr | (data[6] << 8) 
            curr = curr | data[7]
            curr_A = status_negative_value_process_2_byte(curr)/100.

            #print(cmd, id, pos_rad, vel_rps, curr_A)

            if cmd == CAN_PACKET_ID['CAN_PACKET_STATUS']:
                data_added_flag = False
                if len(self.status1) == 0:
                    self.status1.append([id, timestamp, period_msec, pos_rad, vel_rps, curr_A, motor_temp])
                    data_added_flag = True
                else:
                    i = 0
                    for stat1 in self.status1:
                        if stat1[0] == id:
                            period_msec = (timestamp - self.status1[i][1])*1000.
                            self.status1[i] = [id, timestamp, period_msec, pos_rad, vel_rps, curr_A, motor_temp]
                            data_added_flag = True
                            break
                        i += 1

                if data_added_flag is False:
                    self.status1.append([id, timestamp, period_msec, pos_rad, vel_rps, curr_A, motor_temp])
                    data_added_flag = True

            self.status1.sort()
        else:
            if debug_non_status_can_msg_print:
                print("<Non Status CAN Data> Timestamp:{}sec | eid:0x{:08x} | dlc:{} | data:{}".format(timestamp, eid, dlc, list2hex(data)))
        #print(self.status1)   

    def pcan_rx_thread(self):
        # CAN MSG Read Routine
        num = 0
        # try~ except 특정 예외
        try:
            # 무한 반복
            while not self.exitThread_pcan:
                self.ReadMessages()
                num += 1
        # Ctrl + C를 입력할 경우
        except KeyboardInterrupt:
            print('Total Rcv number is {}, Quit to receive'.format(num))

    def ReadMessage(self):
            # We execute the "Read" function of the PCANBasic
            #
            result = self.pc1.Read(PCAN_USBBUS1)

            if result[0] == PCAN_ERROR_OK:
                # We show the received message
                #
                self.ProcessMessage(result[1:])
                
            return result[0]

    def ReadMessages(self):
        result = PCAN_ERROR_OK,
        while (result[0] & PCAN_ERROR_QRCVEMPTY) != PCAN_ERROR_QRCVEMPTY:
            result = self.pc1.Read(PCAN_USBBUS1)
            if result[0] != PCAN_ERROR_QRCVEMPTY:
                self.ProcessMessage(result[1:])
            else:
                if(result[0] != PCAN_ERROR_QRCVEMPTY):  print('ERROR_CODE:{}'.format(hex(result[0])))

    def ProcessMessage(self, *args):
        theMsg = args[0][0]
        itsTimeStamp = args[0][1]    

        newMsg = TPCANMsgFD()
        newMsg.ID = theMsg.ID
        newMsg.DLC = theMsg.LEN
        for i in range(8 if (theMsg.LEN > 8) else theMsg.LEN):
            newMsg.DATA[i] = theMsg.DATA[i]
        newMsg.MSGTYPE = theMsg.MSGTYPE
        newTimestamp = TPCANTimestampFD()
        newTimestamp.value = (itsTimeStamp.micros + 1000 * itsTimeStamp.millis + 0x100000000 * 1000 * itsTimeStamp.millis_overflow)

        time = "Timestamp:{:0.3f}sec".format(newTimestamp.value/1000000)
        period = newTimestamp.value - self.time_prev
        cycle_time = "Cycle Time:{:0.3f}msec".format(period/1000)
        TYPE = "TYPE:{}".format(msg_type[hex(newMsg.MSGTYPE)])
        EID = "EID:{:08x}h".format(newMsg.ID)
        DLC = "DLC:{}".format(newMsg.DLC)
        DATA = ' '.join('{:02x}'.format(newMsg.DATA[i]) for i in range(newMsg.DLC))
        DATA_list = []
        for i in range(newMsg.DLC):
            DATA_list.append(newMsg.DATA[i])

        #if newMsg.MSGTYPE == 0x02:  # PCAN_MESSAGE_EXTEND 
        #    print(time,"|",TYPE,"|",EID,"|",DLC,"|",DATA,"|",cycle_time)

        msg = [newMsg.ID, newMsg.DLC, newTimestamp.value/1000000, DATA_list]
        if newMsg.MSGTYPE == 0x02: self.process_vesc_status_msg(msg)
        
        time_prev = newTimestamp.value

######################### USB #########################
def list_serial():
    ports = list(serial.tools.list_ports.comports())
    
    ports_hwid = []
    ports_name = []
    ports_desc = []
    for p in ports:
        if p.vid == 0x0483:
            ports_name.append(p.device)
            ports_desc.append(p.description)
            vid_pid = "{:04x}:{:04x}".format(p.vid, p.pid)
            ports_hwid.append(vid_pid)
    return ports_name, ports_desc, ports_hwid, len(ports_hwid)

def packet_encoding(comm, comm_value = None):
        start_frame = [2]

        if comm == COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']:
            command = comm_value[0]
            vesc_target_id = comm_value[1]
            comm_value = comm_value[2]
            command_frame = [comm, OPENROBOT_HOST_TYPE['USB'], 1, vesc_target_id, command]
        elif comm == COMM_PACKET_ID['COMM_FORWARD_CAN']:
            vesc_target_id = comm_value[0]
            command = comm_value[1]
            comm_value = comm_value[2]
            command_frame = [comm, vesc_target_id, command]
        else:
            command = comm
            command_frame = [command]

        data_list = []
        value = None
        if command == COMM_PACKET_ID['COMM_SET_DUTY']:
            value = int(comm_value * 100000.0)
        elif command == COMM_PACKET_ID['COMM_SET_CURRENT']:
            value = int(comm_value * 1000.0)
        elif command == COMM_PACKET_ID['COMM_SET_CURRENT_BRAKE']:
            value = int(comm_value * 1000.0)
        elif command == COMM_PACKET_ID['COMM_SET_RPM']:
            value = int(comm_value)
        elif command == COMM_PACKET_ID['COMM_SET_POS']:
            value = int(comm_value * 1000000.0)
        elif command == COMM_PACKET_ID_OPENROBOT['COMM_SET_DPS']:
            value = int(comm_value * 1000.0)
        elif command == COMM_PACKET_ID_OPENROBOT['COMM_SET_DPS_VMAX']:
            value = int(comm_value * 1000.0)
        elif command == COMM_PACKET_ID_OPENROBOT['COMM_SET_DPS_AMAX']:
            value = int(comm_value * 1000.0)
        elif command == COMM_PACKET_ID_OPENROBOT['COMM_SET_SERVO']:
            value = int(comm_value * 1000.0)
        elif command == COMM_PACKET_ID_OPENROBOT['COMM_SET_TRAJ']:
            value = int(comm_value * 1000.0)
        elif command == COMM_PACKET_ID['COMM_TERMINAL_CMD']:
            comm_value_bytes = comm_value.encode('utf-8')
            for i in range(len(comm_value_bytes)):
                data_list.append(comm_value_bytes[i])
        else:
            value = None

        if value is not None:
            d1 = (value >> 24) & 0xFF
            d2 = (value >> 16) & 0xFF
            d3 = (value >> 8) & 0xFF
            d4 = value & 0xFF
            data_list = [d1, d2, d3, d4]
        
        data_frame = command_frame + data_list
        data_len = [len(data_frame)]

        #print("data_frame:",data_frame)
        #print("data_len:",data_len)

        arr = (ctypes.c_ubyte * len(data_frame))(*data_frame)
        #crc = crc_vesc.crc16(arr,len(data_frame))
        crc = vc.crc16(arr,len(data_frame))
        crch = (crc >> 8) & 0xFF
        crcl = crc & 0xFF
        crc_frame = [crch, crcl]
        end_frame = [3]
        data_send = start_frame + data_len + data_frame + crc_frame + end_frame
        return data_send

class VESC_USB:
    def __init__(self, serial_port_name, serial_port_row_number):
        self.serial_name = serial_port_name

        self.line = [] #라인 단위로 데이터 가져올 리스트 변수
        self.exitThread_usb = False   # 쓰레드 종료용 변수
        self.controller_id_list = []

        self.usb_port_index = serial_port_row_number
        self.usb_connection_flag = True

    def reset_controller_id_list(self):
        del self.controller_id_list[:]

    def get_controller_id_list(self):
        return self.controller_id_list

    #쓰레드 종료용 시그널 함수
    def handler_usb(self, signum, frame):
        self.exitThread_usb = True

    def crc_check(self, data_frame, crc_input):
        arr = (ctypes.c_ubyte * len(data_frame))(*data_frame)
        #crc = crc_vesc.crc16(arr,len(data_frame))
        crc = vc.crc16(arr,len(data_frame))
        crch = (crc >> 8) & 0xFF
        crcl = crc & 0xFF

        #print("{0} {1}",crc_input[0], crch)
        #print("{0} {1}",crc_input[1], crcl)

        if crc_input[0] == crch and crc_input[1] ==crcl:
            #print("crc check - Pass")
            return True
        else:
            print("crc check - Fail")
            print("received crch:{}, crcl:{}, calculated crch:{}, crch{}".format(crc_input[0], crc_input[1],crch,crcl))
            return False

    def get_bytes(self, data, div=1):
        raw_value = int(0)
        length = len(data)
        for i in range(length-1, -1, -1):
            raw_value = raw_value | data[-(i+1)] << 8*i
        #print(hex(raw_value))
        # Negative hex value process
        value = status_negative_value_process_4_byte(raw_value)
        # int value divided by div
        if div == 1:
            result = int(value)
        else:
            result = int(value)/div
        return result

    #데이터 처리할 함수
    def parsing_data(self, data):
        #print("raw data:", data)
        #print("raw data hex:",list2hex(data))
        
        # data frame divide
        ind = 0
        start_byte = data[ind]; ind += 1
        len = data[ind]; ind += 1
        data_frame = data[ind:-3];
        crc_frame = data[-3:-1];
        end_byte = data[-1]

        #print(start_byte)
        #print(len)
        #print(data_frame)
        #print(crc_frame)
        #print(end_byte)

        # crc_check
        crc_result = self.crc_check(data_frame, crc_frame)
        
        # data parsing
        if start_byte == 2 and end_byte == 3 and crc_result:
            ind_f = 0
            ind_r = len
            command = data_frame[ind_f]; ind_f += 1

            if command == COMM_PACKET_ID['COMM_FW_VERSION']:
                fw_major = data_frame[ind_f]; ind_f += 1
                fw_minor = data_frame[ind_f]; ind_f += 1
                print('VESC Firmware Ver:{0}.{1}'.format(fw_major,fw_minor))

                ind_r -= 1
                hw_type_vesc = data_frame[ind_r]; ind_r -= 1
                fw_test_ver = data_frame[ind_r]; ind_r -= 1
                pairing_done = data_frame[ind_r]; ind_r -= 1
                uuid = data_frame[ind_r-12:ind_r]; ind_r -= 12
                hw_name = data_frame[ind_f:ind_r]

                print("HW_NAME:",list2chr(hw_name))
                print("UUID:",list2hex_nospace(uuid))
                print("pairing_done:",pairing_done)
                print("fw_test_ver:",fw_test_ver)
                print("hw_type_vesc:",hw_type_vesc)
        
            elif command == COMM_PACKET_ID['COMM_PING_CAN']:
                can_connected_id = []
                for i in range(len-1):
                    can_connected_id.append(data_frame[ind_f]); ind_f += 1
                #print("can connected ID:",can_connected_id)
                self.controller_id_list = self.controller_id_list + can_connected_id
                print(self.serial_name.port,"- IDs:",self.controller_id_list)

            elif command == COMM_PACKET_ID['COMM_GET_VALUES']:
                temp_fet = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                temp_motor = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                motor_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                input_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                id_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                iq_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                duty = self.get_bytes(data_frame[ind_f:ind_f+2], 1000); ind_f += 2
                erpm = self.get_bytes(data_frame[ind_f:ind_f+4], 1); ind_f += 4
                volt_input = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                amp_hours = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                amp_hours_charged = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                watt_hours = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                watt_hours_charged = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                tacho = self.get_bytes(data_frame[ind_f:ind_f+4], 1); ind_f += 4
                tacho_abs = self.get_bytes(data_frame[ind_f:ind_f+4], 1); ind_f += 4
                fault = self.get_bytes(data_frame[ind_f:ind_f+1]); ind_f += 1
                pid_pos_now = self.get_bytes(data_frame[ind_f:ind_f+4], 1000000); ind_f += 4
                controller_id = self.get_bytes(data_frame[ind_f:ind_f+1]); ind_f += 1
                temp_mos1 = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                temp_mos2 = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                temp_mos3 = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                vd_volt = self.get_bytes(data_frame[ind_f:ind_f+4], 1000); ind_f += 4
                vq_volt = self.get_bytes(data_frame[ind_f:ind_f+4], 1000); ind_f += 4           

                self.controller_id_list.append(controller_id)

                if debug_print_get_value_return:
                    print("Controller Id:",controller_id)
                    print("temp_fet:",temp_fet,"'C")
                    print("temp_motor:",temp_motor,"'C")
                    print("motor_current:",motor_current,"A")
                    print("input_current:",input_current,"A")
                    print("id_current:",id_current,"A")
                    print("iq_current:",iq_current,"A")
                    print("duty:",duty)
                    print("erpm:",erpm)
                    print("volt_input:",volt_input,"V")
                    print("amp_hours:",amp_hours,"Ah")
                    print("amp_hours_charged:",amp_hours_charged,"Ah")
                    print("watt_hours:",watt_hours,"Wh")
                    print("watt_hours_charged:",watt_hours_charged,"Wh")
                    print("tacho:",tacho)
                    print("tacho_abs:",tacho_abs)
                    print("fault:",fault)
                    print("pid_pos_now:",pid_pos_now,"deg")
                    print("controller_id:",controller_id)
                    print("temp_mos1:",temp_mos1,"'C")
                    print("temp_mos2:",temp_mos2,"'C")
                    print("temp_mos3:",temp_mos3,"'C")
                    print("vd_volt:",vd_volt,"V")
                    print("vq_volt:",vq_volt,"V")      

            elif command == COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']:
                #print("custom")
                #print("raw data hex:",list2hex(data_frame))

                can_devs_num = data_frame[ind_f]; ind_f += 1
                controller_id = data_frame[ind_f]; ind_f += 1
                volt_input = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                temp_fet = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                temp_motor = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                motor_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                input_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                duty = self.get_bytes(data_frame[ind_f:ind_f+2], 1000); ind_f += 2
                watt_hours = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                watt_hours_charged = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                accum_pos_now = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                rps = self.get_bytes(data_frame[ind_f:ind_f+2], 100); ind_f += 2

                if debug_print_custom_return:
                    print("Controller Id:",controller_id)
                    print("temp_fet:",temp_fet,"'C")
                    print("temp_motor:",temp_motor,"'C")
                    print("motor_current:",motor_current,"A")
                    print("input_current:",input_current,"A")
                    print("duty:",duty)
                    print("volt_input:",volt_input,"V")
                    print("watt_hours:",watt_hours,"Wh")
                    print("watt_hours_charged:",watt_hours_charged,"Wh")
                    print("accum_pos_now:",accum_pos_now,"deg")
                    print("speed:",rps,"rps")
                    print("CAN Connected Device Number:",can_devs_num)
                    print("==============================================")

                for i in range(can_devs_num):
                    controller_id = data_frame[ind_f]; ind_f += 1
                    temp_motor = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                    motor_current = self.get_bytes(data_frame[ind_f:ind_f+2], 100); ind_f += 2
                    accum_pos_now = self.get_bytes(data_frame[ind_f:ind_f+2], 1000); ind_f += 2
                    rps = self.get_bytes(data_frame[ind_f:ind_f+2], 100); ind_f += 2

                    if debug_print_custom_return:
                        print("Can Connected Controller Id:",controller_id)
                        print("temp_motor:",temp_motor,"'C")
                        print("motor_current:",motor_current,"A")
                        print("accum_pos_now:",accum_pos_now,"deg")
                        print("speed:",rps,"rps")
                        print("-------------------------------------------")

            elif command == COMM_PACKET_ID['COMM_PRINT']:
                print(bytes(data_frame).decode())

    # USB RX Thread
    def readThread(self):
        packet_start_flag = 0
        index = 0
        length = 0

        # 쓰레드 종료될때까지 계속 돌림
        while not self.exitThread_usb:
            #데이터가 있있다면
            for c in self.serial_name.read():           
                #line 변수에 차곡차곡 추가하여 넣는다.
                if c == 2 and packet_start_flag == 0:
                    # start to recv
                    packet_start_flag = 1
                
                if packet_start_flag == 1:
                    # start byte
                    self.line.append(c)
                    packet_start_flag = 2
                elif packet_start_flag == 2:
                    # length byte
                    length = c + 3
                    self.line.append(c)
                    packet_start_flag = 3
                elif packet_start_flag == 3:
                    # remained bytes
                    self.line.append(c)
                    index += 1

                    #print("c:",c)
                    #print("index:",index)
                    #print("length:",length)

                    if c == 3 and length==index:
                        packet_start_flag = 0
                        index = 0
                        length = 0
                        self.parsing_data(self.line)
                        del self.line[:]      

    def serial_write(self, data):
        self.serial_name.write(data)