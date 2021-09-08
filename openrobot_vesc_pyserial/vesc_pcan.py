from .general_defines import *
from .PCANBasic import *

######################### PCAN #########################
class PCAN:
    def __init__(self):
        self.status1 = []

        self.exitThread_pcan = False   # 쓰레드 종료용 변수
        self.pcan_connection_flag = False

        self.time_prev = 0
        self.pc1 = PCANBasic()

        self.debug_non_status_can_msg_print = False

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
            temperature = (eid >> 16) & 0xFFFF
            motor_temp = status_negative_value_process_2_byte(temperature)/10. - 273.0
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

            #print(cmd, id, pos_rad, vel_rps, curr_A, motor_temp)

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
            if self.debug_non_status_can_msg_print:
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

