from tkinter.constants import NONE
import PySimpleGUI as sg
import ctypes
import platform
from PySimpleGUI.PySimpleGUI import T
import serial
import time
import vesc_pyserial as vs
from threading import Timer

######################### Class Init #########################
# pcan
pcan = vs.PCAN()

# default serial
selected_ser_name = ''
selected_ser_class = ''
ser_ind = 0
baud = ('115200', '460800', '921600')
tout = 0.5

# default about joint
joint_list = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
joint_original_tuple = tuple(joint_list)
vesc_joint_match = {1:'J1', 2:'J2', 3:'J3', 4:'J4', 5:'J5', 6:'J6'} # dictionary type, ID:'Joint Number'
global vesc_id_data
refresh_list_flag = False
vesc_serial_list = []

# gui theme select
sg.theme('DarkAmber')   # Add a touch of color

def get_serial_port_name_from_class(s_class):
    return s_class.serial_name.port

def get_serial_class_from_port_name(port_name):
    vesc_serial_class_selected = None
    for class_instance in vesc_serial_list:
        if get_serial_port_name_from_class(class_instance) == port_name:
            vesc_serial_class_selected = class_instance
            #print("selected vesc serial class instance:",vesc_serial_class_selected)
    return vesc_serial_class_selected    

def del_serial_class(s_class):
    i = 0
    for class_instance in vesc_serial_list:
        if class_instance == s_class:
            del vesc_serial_list[i]
            #print("selected vesc serial class instance:",vesc_serial_class_selected)
        i += 1

def del_vesc_list(port_name):
    i = 0
    for i in range(len(vesc_id_data)-1, -1, -1):
        vesc_obj = vesc_id_data[i]
        if vesc_obj[0] == port_name:
            vesc_id_data.remove(vesc_obj)

def get_vesc_id_from_joint_number(joint_num):
    print(joint_num)

def refresh_joint_list_after_delete_seleced_joint_number(joint_num):
    for i in range(len(joint_list)-1, -1, -1):
        if joint_list[i] == joint_num:
            joint_list.remove(joint_num)
    joint_list.sort()    

def refresh_joint_list_after_replace_seleced_joint_number(prev_joint_num, selected_joint_num):
    for i in range(len(joint_list)-1, -1, -1):
        if joint_list[i] == selected_joint_num:
            joint_list.remove(selected_joint_num)
    joint_list.append(prev_joint_num)
    joint_list.sort()    

def reset_joint_list():
    global joint_list
    joint_list = list(joint_original_tuple)
    #print("joint_list",joint_list)

######################### Common Functions #########################
# For 4K monitor(HiDPI)
def make_dpi_aware():
    if platform.system() == "Windows":
        if int(platform.release()) == "7":
            ctypes.windll.user32.SetProcessDPIAware()
        elif int(platform.release()) >= 8:
            ctypes.windll.shcore.SetProcessDpiAwareness(True)

# 시리얼 포트 스캔
def scan_serial_ports():
    p_name, p_desc, p_hwid, p_num = vs.list_serial()
    #print(p_name)
    #print(p_desc)
    #print(p_hwid)
    if p_num == 0: p_data = [["No Device"],[""]]
    else:
        p_data = [] 
        for i in range(p_num):
            p_data = p_data + [[p_name[i], p_hwid[i]]]
    return p_data

def refresh_vesc_list():
    vesc_id_data.clear()
    for ser_class in vesc_serial_list:
        if ser_class is not None:
            ids = ser_class.get_controller_id_list()
            
            if len(ids) != 0:
                for i in range(len(ids)):
                    try:
                        if i == 0:
                            if vesc_joint_match[ids[i]] is not None: 
                                vesc_id_data.append([get_serial_port_name_from_class(ser_class), ids[i], "Local", vesc_joint_match[ids[i]]])
                        else:
                            if vesc_joint_match[ids[i]] is not None: 
                                vesc_id_data.append([get_serial_port_name_from_class(ser_class), ids[i], "CAN", vesc_joint_match[ids[i]]])
                    except:
                        if i == 0:
                            vesc_id_data.append([get_serial_port_name_from_class(ser_class), ids[i], "Local", joint_list[i]])
                            print("Joint number of vesc id:", ids[i], "is not pre-defined")
                        else:
                            vesc_id_data.append([get_serial_port_name_from_class(ser_class), ids[i], "CAN", joint_list[i]])
                            print("Joint number of vesc id:", ids[i], "is not pre-defined")
                #print(vesc_id_data)
        else:
            print("No devices, SCAN VESC first")
    window.Element('-VESC_TABLE-').Update(values=vesc_id_data)
    refresh_list_flag = False

def set_servo_control(vesc_target_id, value):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_cmd = vs.COMM_PACKET_ID_OPENROBOT['COMM_SET_SERVO']

    custom_data = [custom_cmd, vesc_target_id, value]
    send_data = vs.packet_encoding(comm_set_cmd, custom_data)
    return send_data

def set_release(vesc_target_id):
    # prepare data
    comm_set_cmd = vs.COMM_PACKET_ID['COMM_CUSTOM_APP_DATA']
    custom_cmd = vs.COMM_PACKET_ID_OPENROBOT['COMM_SET_RELEASE']

    custom_data = [custom_cmd, vesc_target_id, 0]
    send_data = vs.packet_encoding(comm_set_cmd, custom_data)
    return send_data

def send_cmd(joint_number, cmd, value=0):
    s_class = None
    for i in range(len(vesc_id_data)):
        if len(vesc_id_data[i]) == 4 and vesc_id_data[i][3] == joint_number:
            s_class = get_serial_class_from_port_name(vesc_id_data[i][0])
            vesc_id = vesc_id_data[i][1]
            break

    if s_class is not None and s_class.usb_connection_flag:
        if vesc_id_data[i][2] == 'Local':
                vesc_id = 0xFF
        if cmd == "release":
            send_data = set_release(vesc_id)
            s_class.serial_write(send_data)
            print(joint_number, "Released")
        elif cmd == "servo":
            send_data = set_servo_control(vesc_id, value)
            s_class.serial_write(send_data)
        elif cmd == "terminal":
            # terminal command using CAN_FORWARD
            if vesc_id ==  0xFF:
                comm_set_cmd = vs.COMM_PACKET_ID['COMM_TERMINAL_CMD']
                send_data = vs.packet_encoding(comm_set_cmd, value)
                s_class.serial_write(send_data)
            else:
                comm_set_cmd = vs.COMM_PACKET_ID['COMM_FORWARD_CAN']
                send_data = vs.packet_encoding(comm_set_cmd, [vesc_id, vs.COMM_PACKET_ID['COMM_TERMINAL_CMD'], value])
                s_class.serial_write(send_data)
    else:
        print(joint_number, "is Not Connected")

def adjust_joint_limit(joint_range_i, joint_range_j, keyhere):
    layout_subwin2_1 = [ [sg.Text(keyhere)],
                         [sg.InputText(joint_range[joint_range_i][joint_range_j], key=keyhere), sg.Button('Ok', bind_return_key=True)]
    ]
    window_subwin2_1 = sg.Window('Joint Limit Reset', layout_subwin2_1)
    events_subwin2_1, values_subwin2_1 = window_subwin2_1.Read()

    if events_subwin2_1 == 'Ok':
        try:
            value = int(values_subwin2_1[keyhere])
            #print(type(value), value)
            joint_range[joint_range_i][joint_range_j] = value
            window.Element(event).Update(values_subwin2_1[keyhere])
            window.Refresh()
        except:
            print("Please input number")
    window_subwin2_1.close()

######################### GUI #########################
# usb port table data 
ports_title = ["Port Name", "VID:PID"]
ports_data = scan_serial_ports()
# vesc id table data
vesc_id_data = []
vesc_id_headings = ["Port","ID","COMM", "Joint"]
layout_col1 = [ [sg.Text('<VESC USB>', font=("Tahoma", 20))],
                [sg.Table(values=ports_data, headings=ports_title, max_col_width=100,
                                col_widths=[12,11],
                                background_color='black',
                                auto_size_columns=False,
                                display_row_numbers=True,
                                justification='center',
                                num_rows=3,
                                alternating_row_color='black',
                                enable_events=True,
                                key='-USB_TABLE-',
                                row_height=40)],
                [sg.Text('Baudrate', font=("Tahoma", 12)), sg.Combo(values=baud, default_value='921600', readonly=True, k='-BAUD-')],
                [sg.Button('SCAN'), sg.Button('CONNECT'), sg.Button('DISCONNECT')],
                [sg.HorizontalSeparator()],
                [sg.Text('Connected VESC IDs')],
                [sg.Table(values=vesc_id_data, headings=vesc_id_headings, max_col_width=100,
                                col_widths=[10,5,7,6],
                                background_color='black',
                                bind_return_key=True,
                                auto_size_columns=False,
                                display_row_numbers=False,
                                justification='center',
                                num_rows=8,
                                alternating_row_color='black',
                                key='-VESC_TABLE-',
                                row_height=40)],
                [sg.Button('SCAN VESC'), sg.Button('Refresh List')],
                [sg.HorizontalSeparator()],
                [sg.Text('<PCAN>', font=("Tahoma", 20))],
                [sg.Button('PCAN OPEN'), sg.Button('PCAN CLOSE')],
                [sg.Text('PCAN disconnected')],
]

joint_max = 1800
joint_min = -1800
joint_range = [[joint_min, joint_max], [joint_min, joint_max], [joint_min, joint_max], [joint_min, joint_max], [joint_min, joint_max], [joint_min, joint_max]]
task_range = (-100, 100)
angle_range = (-90, 90)
default_pos = 0
size_bar_joint = (21.5,20)
size_bar_task = (30,20)
layout_col2 = [ [sg.Text('<Joint Space Control [deg]>', font=("Tahoma", 17))],
                [sg.Text(joint_list[0]), sg.Text(" "), 
                 sg.Text(joint_range[0][0], enable_events=True, key='-JOINT1_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[0][0], joint_range[0][1]], default_value = default_pos, size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT1-'), 
                 sg.Text(joint_range[0][1], enable_events=True, key='-JOINT1_MAX-', size=(8,1)), sg.Button('J1 Release')],
                [sg.Text(joint_list[1]), sg.Text(" "), 
                 sg.Text(joint_range[1][0], enable_events=True, key='-JOINT2_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[1][0], joint_range[1][1]], default_value = default_pos, size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT2-'), 
                 sg.Text(joint_range[1][1], enable_events=True, key='-JOINT2_MAX-', size=(8,1)), sg.Button('J2 Release')],
                [sg.Text(joint_list[2]), sg.Text(" "), 
                 sg.Text(joint_range[2][0], enable_events=True, key='-JOINT3_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[2][0], joint_range[2][1]], default_value = default_pos, size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT3-'), 
                 sg.Text(joint_range[2][1], enable_events=True, key='-JOINT3_MAX-', size=(8,1)), sg.Button('J3 Release')],
                [sg.Text(joint_list[3]), sg.Text(" "), 
                 sg.Text(joint_range[3][0], enable_events=True, key='-JOINT4_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[3][0], joint_range[3][1]], default_value = default_pos, size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT4-'), 
                 sg.Text(joint_range[3][1], enable_events=True, key='-JOINT4_MAX-', size=(8,1)), sg.Button('J4 Release')],
                [sg.Text(joint_list[4]), sg.Text(" "), 
                 sg.Text(joint_range[4][0], enable_events=True, key='-JOINT5_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[4][0], joint_range[4][1]], default_value = default_pos, size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT5-'), 
                 sg.Text(joint_range[4][1], enable_events=True, key='-JOINT5_MAX-', size=(8,1)), sg.Button('J5 Release')],
                [sg.Text(joint_list[5]), sg.Text(" "), 
                 sg.Text(joint_range[5][0], enable_events=True, key='-JOINT6_MIN-', size=(8,1)), 
                 sg.Slider(range = [joint_range[5][0], joint_range[5][1]], default_value = default_pos, size = size_bar_joint, orientation='h', enable_events=True, key='-JOINT6-'), 
                 sg.Text(joint_range[5][1], enable_events=True, key='-JOINT6_MAX-', size=(8,1)), sg.Button('J6 Release')],
                [sg.Button('IK ON'), sg.Button('IK OFF'), sg.Button('Gripper OPEN'), sg.Button('Gripper CLOSE'), sg.Button('All Release')],
                [sg.HorizontalSeparator()],
                [sg.Text('<Task Space Control [mm, deg]>', font=("Tahoma", 17))],
                [sg.Text('[X]'), sg.Text(" "), sg.Text(task_range[0]), sg.Slider(range = task_range, default_value = default_pos, size = size_bar_task, orientation='h', enable_events=True, key='-X-'), sg.Text(task_range[1])],
                [sg.Text('[Y]'), sg.Text(" "), sg.Text(task_range[0]), sg.Slider(range = task_range, default_value = default_pos, size = size_bar_task, orientation='h', enable_events=True, key='-Y-'), sg.Text(task_range[1])],
                [sg.Text('[Z]'), sg.Text(" "), sg.Text(task_range[0]), sg.Slider(range = task_range, default_value = default_pos, size = size_bar_task, orientation='h', enable_events=True, key='-Z-'), sg.Text(task_range[1])],
                [sg.Text('[R]'), sg.Text(" "), sg.Text(angle_range[0]), sg.Slider(range = angle_range, default_value = default_pos, size = size_bar_task, orientation='h', enable_events=True, key='-ROLL-'), sg.Text(angle_range[1])],
                [sg.Text('[P]'), sg.Text(" "), sg.Text(angle_range[0]), sg.Slider(range = angle_range, default_value = default_pos, size = size_bar_task, orientation='h', enable_events=True, key='-PITCH-'), sg.Text(angle_range[1])],
                [sg.Text('[Y]'), sg.Text(" "), sg.Text(angle_range[0]), sg.Slider(range = angle_range, default_value = default_pos, size = size_bar_task, orientation='h', enable_events=True, key='-YAW-'), sg.Text(angle_range[1])],
]

status_data = []
status_heading = ["ID", "msec", "pos_rad", "vel_rps", "curr_A", "motor_temp"]
size_input = (10,None)
via_data = [["Via1", "(10, 20, 30, 12, 23, 31)", "(10, 20, 30, 12, 23, 31)"], ["Via2", "(14, 23, 13, 23, 32, 13)", "(14, 23, 13, 23, 32, 13)"]]
via_headings = ["Via Point", "Joint Data", "Task Data"]
layout_col3 = [ [sg.Text('<Status>', font=("Tahoma", 17))],
                [sg.Table(values=status_data, headings=status_heading, max_col_width=100,
                                background_color='black',
                                col_widths=[5,6,8,8,8,9],
                                auto_size_columns=False,
                                display_row_numbers=False,
                                justification='center',
                                num_rows=8,
                                alternating_row_color='black',
                                key='-STATUS_TABLE-',
                                row_height=40)],
                [sg.HorizontalSeparator()],
                [sg.Text('<Control>', font=("Tahoma", 17))],
                [sg.Button('Finding Home'), sg.Button('Set Current as Home'), sg.Button('Go Home'), sg.Button('Remote Control')],
                [sg.Button('Clear Via Point'), sg.Button('Add Via Point'), sg.Button('Play Via Point')],
                [sg.Table(values=via_data, headings=via_headings, max_col_width=150,
                                background_color='black',
                                col_widths=[7,17,17],
                                auto_size_columns=False,
                                display_row_numbers=True,
                                justification='right',
                                num_rows=10,
                                alternating_row_color='black',
                                key='-TABLE_VIA-',
                                row_height=40)]
]

logging_layout = [ [sg.Text("Anything printed will display here!"), 
                    sg.Button('Clear'),
                    sg.Button('DEBUG PRINT ON'), 
                    sg.Button('DEBUG PRINT OFF'),
                    sg.Button('NON Status CAN MSG Print ON'),
                    sg.Button('NON Status CAN MSG Print OFF')],
                   [sg.Output(size=(162,20), font='Courier 8', key='-OUTPUT-')],
                   [sg.Text("Terminal Command to"), 
                    sg.Combo(values=joint_list, default_value=joint_list[0], readonly=True, k='-TARGET_JOINT-'), 
                    sg.Input(size=(109,1), focus=True, key='-TERMINAL_INPUT-'), sg.Button('Terminal SEND', bind_return_key=True)],
]
layout_main = [ [sg.Column(layout_col1), 
                 sg.VSeparator(), 
                 sg.Column(layout_col2), 
                 sg.VSeparator(), 
                 sg.Column(layout_col3),
                ],
                [sg.HSeparator()],
                [sg.Column(logging_layout)],
]

# 4k hidpi solution
make_dpi_aware()
#print(platform.architecture())
# Create the Window
window = sg.Window('Window Title', layout_main)

######################### GUI Event Loop #########################
# Event Loop to process "events" and get the "values" of the inputs
while True:
    event, values = window.read(timeout=100)

    if event == sg.WIN_CLOSED or event == 'Cancel': # if user closes window or clicks cancel
        for class_instance in vesc_serial_list:
            class_instance.exitThread_usb = True
            class_instance.usb_connection_flag = False
            # close serial
            if class_instance.serial_name is not None:
                if class_instance.serial_name.is_open:
                    class_instance.serial_name.close()

        # close pcan
        pcan.exitThread_pcan = True
        if pcan.pcan_connection_flag:
            result = pcan.pcan_Close(False)
        break
    else:
        #pcan status data
        st_data = pcan.get_status1()
        window.Element('-STATUS_TABLE-').Update(values=st_data)

    if event == "PCAN OPEN":
        result = pcan.pcan_Open()
        if result == 0: 
            pcan.exitThread_pcan = False
            vs.signal.signal(vs.signal.SIGINT, pcan.handler_pcan)

            #시리얼 읽을 쓰레드 생성
            thread_pcan_rx = vs.threading.Thread(target=pcan.pcan_rx_thread)
            #시작!
            thread_pcan_rx.start()
            print("PCAN RX Thread Started")
        else:
            print("PCAN is not connected")

    if event == "PCAN CLOSE":
        pcan.exitThread_pcan = True
        result = pcan.pcan_Close()

    if event == "-USB_TABLE-":
        row = values['-USB_TABLE-']
        if len(row) != 0:
            ser_ind = row[0]
            selected_ser_name = ports_data[ser_ind][0]
            print("USB Port Selected - row:{0}, port:{1}".format(ser_ind, selected_ser_name))

    if event == "SCAN":
        print("Scan valid usb ports...")
        ports_data = scan_serial_ports()
        window.Element('-USB_TABLE-').Update(values=ports_data)
        
    if event == "Refresh List":
        refresh_vesc_list()

    if event == "CONNECT":
        baudrate_sel = values['-BAUD-']
        if selected_ser_name == 'No Device':
            print("Select valid devices")
        elif selected_ser_name != '':
            if get_serial_class_from_port_name(selected_ser_name) is None:
                while True:
                    try:
                        # create serial connection class
                        selected_ser_class = vs.VESC_USB(serial.Serial(selected_ser_name, baudrate_sel, timeout=tout), ser_ind)
                        vesc_serial_list.append(selected_ser_class)
                    except serial.SerialException:
                        continue
                    else:
                        print("VESC USB Connected at {}, {}bps".format(get_serial_port_name_from_class(selected_ser_class), baudrate_sel))
                        break

                if selected_ser_class.usb_connection_flag:
                    #종료 시그널 등록
                    vs.signal.signal(vs.signal.SIGINT, selected_ser_class.handler_usb)

                    #시리얼 읽을 쓰레드 생성
                    thread_usb_rx = vs.threading.Thread(target=selected_ser_class.readThread)
                    #시작!
                    thread_usb_rx.start()
                    print("VESC USB RX Thread Started", selected_ser_class)

                    # Specify Next event, automatically scan vesc IDs
                    event = "SCAN VESC"
            else:
                print("Select USB Port is already Opened")
        else:
            print("Please select USB Port first")
        
    if event == "DISCONNECT":
        if selected_ser_name == 'No Device':
            print("Select valid devices")
        elif selected_ser_name != '':
            selected_ser_class = get_serial_class_from_port_name(selected_ser_name)
            
            #print(len(vesc_id_data))

            if selected_ser_class is not None:
                #print(vesc_id_data)
                # delete connected vesc list 
                del_vesc_list(get_serial_port_name_from_class(selected_ser_class))
                selected_ser_class.reset_controller_id_list()
                #print(vesc_id_data)
                window.Element('-VESC_TABLE-').Update(values=vesc_id_data)

                # serial class disconnection
                selected_ser_class.exitThread_usb = True
                selected_ser_class.serial_name.close()
                del_serial_class(selected_ser_class)
                print("VESC USB Disconnected")
            else:
                print("VESC Not Connected")

            reset_joint_list()
        else:
            print("Please select USB Port first")
 
    if event == "SCAN VESC":
        selected_ser_class = get_serial_class_from_port_name(selected_ser_name)

        if selected_ser_class is not None:
            if selected_ser_class.usb_connection_flag:
                selected_ser_class.reset_controller_id_list()
                send_data = vs.packet_encoding(vs.COMM_PACKET_ID['COMM_GET_VALUES'])
                #print(send_data)
                selected_ser_class.serial_write(send_data)
                time.sleep(0.1)
                send_data = vs.packet_encoding(vs.COMM_PACKET_ID['COMM_PING_CAN'])
                #print(send_data)
                selected_ser_class.serial_write(send_data)
                
                # 5초후에 REPRESH LIST event 실행하기
                if refresh_list_flag == False:
                    Timer(5, refresh_vesc_list).start()           
            else:
                print("VESC Not Connected")
        else:
            print("VESC Not Connected")

    if event == "Clear":
        window['-OUTPUT-'].update(value='')

    if event == "NON Status CAN MSG Print ON":
        vs.debug_non_status_can_msg_print = True
        print("NON Status CAN MSG Print ON")
    
    if event == "NON Status CAN MSG Print OFF":
        vs.debug_non_status_can_msg_print = False
        print("NON Status CAN MSG Print OFF")

    if event == "DEBUG PRINT ON":
        vs.debug_print_get_value_return = True
        vs.debug_print_custom_return = True
        print("DEBUG PRINT ON")

    if event == "DEBUG PRINT OFF":
        vs.debug_print_get_value_return = False
        vs.debug_print_custom_return = False
        print("DEBUG PRINT OFF")

    if event == "Terminal SEND":
        send_cmd(values['-TARGET_JOINT-'], 'terminal', values['-TERMINAL_INPUT-'])

    if event == "-VESC_TABLE-":
        if len(joint_list) != 0:
            layout_subwin1 = [ [sg.Text('Select Joint Number')],
                               [sg.Combo(values=joint_list, default_value=joint_list[0], readonly=True, k='-JOINT_COMBO-'), sg.Button('Ok')]
            ]
            window_subwin1 = sg.Window('Joint Selection', layout_subwin1)
            events_subwin1, values_subwin1 = window_subwin1.Read()

            if events_subwin1 == 'Ok':
                if values_subwin1['-JOINT_COMBO-']:    # if something is highlighted in the list
                    joint_number = values_subwin1['-JOINT_COMBO-']
                    if len(vesc_id_data[values['-VESC_TABLE-'][0]]) == 3:
                        vesc_id_data[values['-VESC_TABLE-'][0]].append(joint_number)
                        refresh_joint_list_after_delete_seleced_joint_number(joint_number)
                    elif len(vesc_id_data[values['-VESC_TABLE-'][0]]) == 4 and vesc_id_data[values['-VESC_TABLE-'][0]][3] != joint_number:
                        last_joint_number = vesc_id_data[values['-VESC_TABLE-'][0]][3]
                        vesc_id_data[values['-VESC_TABLE-'][0]][3] = joint_number
                        refresh_joint_list_after_replace_seleced_joint_number(last_joint_number, joint_number)

            window_subwin1.close()

            print(vesc_id_data[values['-VESC_TABLE-'][0]])
            window.Element('-VESC_TABLE-').Update(values=vesc_id_data)
        else:
            sg.Popup("All Joint selected")
        
    if event == "-JOINT1-":       
        val = values[event]
        send_cmd('J1', 'servo', val)
    if event == "-JOINT1_MIN-":
        joint_index = 0
        adjust_joint_limit(joint_index,0,'JOINT1 MIN VALUE:')
        window.Element('-JOINT1-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT1_MAX-":
        joint_index = 0
        adjust_joint_limit(joint_index,1,'JOINT1 MAX VALUE:')
        window.Element('-JOINT1-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT2-":
        val = values[event]
        send_cmd('J2', 'servo', val)
    if event == "-JOINT2_MIN-":
        joint_index = 1
        adjust_joint_limit(joint_index,0,'JOINT2 MIN VALUE:')
        window.Element('-JOINT2-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT2_MAX-":
        joint_index = 1
        adjust_joint_limit(joint_index,1,'JOINT2 MAX VALUE:')
        window.Element('-JOINT2-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT3-":
        val = values[event]
        send_cmd('J3', 'servo', val)
    if event == "-JOINT3_MIN-":
        joint_index = 2
        adjust_joint_limit(joint_index,0,'JOINT3 MIN VALUE:')
        window.Element('-JOINT3-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT3_MAX-":
        joint_index = 2
        adjust_joint_limit(joint_index,1,'JOINT3 MAX VALUE:')
        window.Element('-JOINT3-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT4-":
        val = values[event]
        send_cmd('J4', 'servo', val)
    if event == "-JOINT4_MIN-":
        joint_index = 3
        adjust_joint_limit(joint_index,0,'JOINT4 MIN VALUE:')
        window.Element('-JOINT4-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT4_MAX-":
        joint_index = 3
        adjust_joint_limit(joint_index,1,'JOINT4 MAX VALUE:')
        window.Element('-JOINT4-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT5-":
        val = values[event]
        send_cmd('J5', 'servo', val)
    if event == "-JOINT5_MIN-":
        joint_index = 4
        adjust_joint_limit(joint_index,0,'JOINT5 MIN VALUE:')
        window.Element('-JOINT5-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT5_MAX-":
        joint_index = 4
        adjust_joint_limit(joint_index,1,'JOINT5 MAX VALUE:')
        window.Element('-JOINT5-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "-JOINT6-":
        val = values[event]
        send_cmd('J6', 'servo', val)
    if event == "-JOINT6_MIN-":
        joint_index = 5
        adjust_joint_limit(joint_index,0,'JOINT6 MIN VALUE:')
        window.Element('-JOINT6-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])
    if event == "-JOINT6_MAX-":
        joint_index = 5
        adjust_joint_limit(joint_index,1,'JOINT6 MAX VALUE:')
        window.Element('-JOINT6-').Update(range = [joint_range[joint_index][0], joint_range[joint_index][1]])
        print("Joint", joint_index+1, "is Limit adjusted as", joint_range[joint_index][0], "to", joint_range[joint_index][1])

    if event == "All Release":
        for joint in joint_list:
            send_cmd(joint, 'release')

    if event == "J1 Release":
        send_cmd('J1', 'release')

    if event == "J2 Release":
        send_cmd('J2', 'release')

    if event == "J3 Release":
        send_cmd('J3', 'release')

    if event == "J4 Release":
        send_cmd('J4', 'release')

    if event == "J5 Release":
        send_cmd('J5', 'release')

    if event == "J6 Release":
        send_cmd('J6', 'release')