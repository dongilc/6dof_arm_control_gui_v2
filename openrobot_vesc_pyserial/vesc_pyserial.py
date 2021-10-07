import serial
import time
import signal
import threading
import sys
import glob
import ctypes
import serial.tools.list_ports
import platform
from . import vesc_crc
from .general_defines import *

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
        crc = vesc_crc.crc16(arr,len(data_frame))
        crch = (crc >> 8) & 0xFF
        crcl = crc & 0xFF
        crc_frame = [crch, crcl]
        end_frame = [3]
        data_send = start_frame + data_len + data_frame + crc_frame + end_frame
        return data_send

class VESC_VALUES:
    def __init__(self):
        self.temp_fet = 0
        self.temp_motor = 0
        self.otor_current = 0
        self.input_current = 0
        self.id_current = 0
        self.iq_current = 0
        self.duty = 0
        self.erpm = 0
        self.volt_input = 0
        self.amp_hours = 0
        self.amp_hours_charged = 0
        self.watt_hours = 0
        self.watt_hours_charged = 0
        self.tacho = 0
        self.tacho_abs = 0
        self.fault = 0
        self.pid_pos_now = 0
        self.controller_id = 0
        self.temp_mos1 = 0
        self.temp_mos2 = 0
        self.temp_mos3 = 0
        self.vd_volt = 0
        self.vq_volt = 0

class VESC_USB:
    def __init__(self, serial_port_name, serial_port_row_number):
        self.serial_name = serial_port_name

        self.line = [] #라인 단위로 데이터 가져올 리스트 변수
        self.exitThread_usb = False   # 쓰레드 종료용 변수
        self.controller_id_list = []

        self.usb_port_index = serial_port_row_number
        self.usb_connection_flag = True

        self.debug_print_get_value_return = False
        self.debug_print_custom_return = False

        self.values = VESC_VALUES()

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
        crc = vesc_crc.crc16(arr,len(data_frame))
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
        if length == 4:
            value = status_negative_value_process_4_byte(raw_value)
        elif length == 3:
            value = status_negative_value_process_3_byte(raw_value)
        elif length == 2:
            value = status_negative_value_process_2_byte(raw_value)
        else:
            value = raw_value

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
                self.values.temp_fet = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                self.values.temp_motor = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                self.values.motor_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                self.values.input_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                self.values.id_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                self.values.iq_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                self.values.duty = self.get_bytes(data_frame[ind_f:ind_f+2], 1000); ind_f += 2
                self.values.erpm = self.get_bytes(data_frame[ind_f:ind_f+4], 1); ind_f += 4
                self.values.volt_input = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                self.values.amp_hours = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                self.values.amp_hours_charged = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                self.values.watt_hours = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                self.values.watt_hours_charged = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                self.values.tacho = self.get_bytes(data_frame[ind_f:ind_f+4], 1); ind_f += 4
                self.values.tacho_abs = self.get_bytes(data_frame[ind_f:ind_f+4], 1); ind_f += 4
                self.values.fault = self.get_bytes(data_frame[ind_f:ind_f+1]); ind_f += 1
                self.values.pid_pos_now = self.get_bytes(data_frame[ind_f:ind_f+4], 1000000); ind_f += 4
                self.values.controller_id = self.get_bytes(data_frame[ind_f:ind_f+1]); ind_f += 1
                self.values.temp_mos1 = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                self.values.temp_mos2 = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                self.values.temp_mos3 = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                self.values.vd_volt = self.get_bytes(data_frame[ind_f:ind_f+4], 1000); ind_f += 4
                self.values.vq_volt = self.get_bytes(data_frame[ind_f:ind_f+4], 1000); ind_f += 4           

                self.controller_id_list.append(self.values.controller_id)

                if self.debug_print_get_value_return:
                    print("Controller Id:",self.values.controller_id)
                    print("temp_fet:",self.values.temp_fet,"'C")
                    print("temp_motor:",self.values.temp_motor,"'C")
                    print("motor_current:",self.values.motor_current,"A")
                    print("input_current:",self.values.input_current,"A")
                    print("id_current:",self.values.id_current,"A")
                    print("iq_current:",self.values.iq_current,"A")
                    print("duty:",self.values.duty)
                    print("erpm:",self.values.erpm)
                    print("volt_input:",self.values.volt_input,"V")
                    print("amp_hours:",self.values.amp_hours,"Ah")
                    print("amp_hours_charged:",self.values.amp_hours_charged,"Ah")
                    print("watt_hours:",self.values.watt_hours,"Wh")
                    print("watt_hours_charged:",self.values.watt_hours_charged,"Wh")
                    print("tacho:",self.values.tacho)
                    print("tacho_abs:",self.values.tacho_abs)
                    print("fault:",self.values.fault)
                    print("pid_pos_now:",self.values.pid_pos_now,"deg")
                    print("controller_id:",self.values.controller_id)
                    print("temp_mos1:",self.values.temp_mos1,"'C")
                    print("temp_mos2:",self.values.temp_mos2,"'C")
                    print("temp_mos3:",self.values.temp_mos3,"'C")
                    print("vd_volt:",self.values.vd_volt,"V")
                    print("vq_volt:",self.values.vq_volt,"V")      

            elif command == COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']:
                #print("custom")
                #print("raw data hex:",list2hex(data_frame))

                can_devs_num = data_frame[ind_f]; ind_f += 1
                controller_id = data_frame[ind_f]; ind_f += 1
                #volt_input = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                #temp_fet = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                temp_motor = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                motor_current = self.get_bytes(data_frame[ind_f:ind_f+2], 100); ind_f += 2
                #input_current = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                #duty = self.get_bytes(data_frame[ind_f:ind_f+2], 1000); ind_f += 2
                #watt_hours = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                #watt_hours_charged = self.get_bytes(data_frame[ind_f:ind_f+4], 10000); ind_f += 4
                accum_pos_now = self.get_bytes(data_frame[ind_f:ind_f+4], 1); ind_f += 4
                rps = self.get_bytes(data_frame[ind_f:ind_f+2], 100); ind_f += 2

                if self.debug_print_custom_return:
                    # Local vesc
                    print("==============================================")
                    print("CAN Connected Device Number:",can_devs_num)
                    print("-------------------------------------------")
                    print("Local Controller Id:",controller_id)
                    #print("temp_fet:",temp_fet,"'C")
                    print("temp_motor:",temp_motor,"'C")
                    print("motor_current:",motor_current,"A")
                    #print("input_current:",input_current,"A")
                    #print("duty:",duty)
                    #print("volt_input:",volt_input,"V")
                    #print("watt_hours:",watt_hours,"Wh")
                    #print("watt_hours_charged:",watt_hours_charged,"Wh")
                    print("accum_pos_now:",accum_pos_now,"deg")
                    print("speed:",rps,"rps")
                    
                for i in range(can_devs_num):
                    # CAN connected vesc
                    controller_id = data_frame[ind_f]; ind_f += 1
                    temp_motor = self.get_bytes(data_frame[ind_f:ind_f+2], 10); ind_f += 2
                    motor_current = self.get_bytes(data_frame[ind_f:ind_f+2], 100); ind_f += 2
                    accum_pos_now = self.get_bytes(data_frame[ind_f:ind_f+4], 100); ind_f += 4
                    rps = self.get_bytes(data_frame[ind_f:ind_f+2], 100); ind_f += 2

                    if self.debug_print_custom_return:
                        print("-------------------------------------------")
                        print("Can Connected Controller Id:",controller_id)
                        print("temp_motor:",temp_motor,"'C")
                        print("motor_current:",motor_current,"A")
                        print("accum_pos_now:",accum_pos_now,"deg")
                        print("speed:",rps,"rps")

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