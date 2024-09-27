import sys
import os
import serial
import socket
import time
import argparse
from wiced_hci_default_config import *
from datetime import datetime
from threading import Thread
from queue import Queue

version = "v1.0"
HCI_CONTROL_GROUP_DEVICE = 0x00

SPY_TRACE_TYPE_TEXT         = 0
SPY_TRACE_TYPE_ERROR        = 1
SPY_TRACE_TYPE_HCI_COMMAND  = 3
SPY_TRACE_TYPE_HCI_EVENT    = 4
SPY_TRACE_TYPE_ACL_RX       = 6
SPY_TRACE_TYPE_ACL_TX       = 7
SPY_TRACE_TYPE_LMP_RECV     = 8
SPY_TRACE_TYPE_LMP_XMIT     = 9

sock = socket.socket()
evtQ = Queue()
scriptInstance = 0

old_stdout = sys.stdout
logfile = None
uart = serial.Serial()
bUseUart = False

class myLog:
    nl = True

    def write(self, x):
        if x == '\n':
            old_stdout.write(x)
            if (logfile != None):
                logfile.write(x)
                logfile.flush()
            self.nl = True
        elif self.nl:
            now = datetime.now().time()
            old_stdout.write('%s>  %s' % (now, x))
            TraceToSpy ("PYTHON==> " + x)
            if (logfile != None):
                logfile.write('%s>  %s' % (now, x))
                logfile.flush()
            self.nl = False
        else:
            old_stdout.write(x)
            if (logfile != None):
                logfile.write(x)
                logfile.flush()

    def flush(self):
        zzz = 0

def InitWicedHCI(logFilePath = None):
    global scriptInstance
    global sock
    global logfile
    global uart
    global bUseUart

    parser = argparse.ArgumentParser(description='Parser')
    parser.add_argument('-com', action="store", dest='port', default=0, type=int)
    parser.add_argument('-log', action="store", dest='logfile', default=None)
    parser.add_argument('-baud', action="store", dest='baud', default=0, type=int)
    parser.add_argument('-inst', action="store", dest='instance', default=-1, type=int)
    args, unknown = parser.parse_known_args()

    if (args.port != 0):
        comPort = args.port
    else:
        comPort = DEFAULT_COMPORT

    if (args.baud != 0):
        baudRate = args.baud
    else:
        baudRate = DEFAULT_BAUDRATE

    if (args.instance != -1):
        scriptInstance = args.instance
    else:
        scriptInstance = DEFAULT_SCRIPT_INSTANCE

    if (args.logfile != None):
        logFilePath = args.logfile

    if (logFilePath != None):
        print ("InitWicedHCI: COM Port {0:d}  BaudRate: {1:d}  Instance: {2:d}  Log File: {3:s}".format(comPort, baudRate, scriptInstance, logFilePath))
    else:
        print ("InitWicedHCI: COM Port {0:d}  BaudRate: {1:d}  Instance: {2:d}  Log File: None".format(comPort, baudRate, scriptInstance))

    sys.stdout = myLog()
    if (logFilePath != None):
        logfile = open(logFilePath, "w")

    if (comPort == 0):              # Use a TCP/IP socket (Emulator or via Tcp2Com)
        server_name  = 'localhost'
        port         = 12012 + scriptInstance

        server_address = (server_name, port)

        print('Connecting TCP to %s port %s' % server_address)

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        sock.connect(server_address)
        print('connected to %s port %s' % server_address)
    else:
        uart.port     = "COM{0:d}".format(comPort)
        uart.baudrate = baudRate
        uart.timeout  = 0.05  # 50ms
        bUseUart      = True

        try:
            uart.open()
        except:
            print ("Failed to open COM Port {0:s} !! Run your script with '-com nn' to select a different  COM port.".format(uart.port))
            uart = None
            os._exit(1) # exit with error exit code

        uart.reset_input_buffer()

    thread = Thread(target = receive_thread, args = ())
    thread.daemon = True
    thread.start()
    time.sleep(0.1)


def sendWicedHciCommand(cmd_code, packet = None):
    global sock
    global uart
    global bUseUart

    bytesdata = bytes([0x19, cmd_code, HCI_CONTROL_GROUP_DEVICE])

    if (packet != None):
        length = len(packet)
        lengthBytes = bytes([length & 0x00FF,(length >> 8)])
        output = bytesdata + lengthBytes + packet
    else:
        lengthBytes = bytes([0,0])
        output = bytesdata + lengthBytes

    if (bUseUart):
        uart.write (output)
    else:
        sock.sendall(output)

def receive_from_socket():
    global sock

    amount_expected = 5
    header = sock.recv(amount_expected)
    length = int.from_bytes(header[3:], byteorder='little')
    amount_expected = length
    amount_received = 0
    message = header
    while amount_received < length:
        packet = sock.recv(amount_expected)
        amount_received += len(packet)
        amount_expected -= len(packet)
        message = message + packet
    return message

def receive_from_uart():
    global uart
    global scriptInstance

    offset = 0
    msg_len = 5
    ba = bytearray(2048)

    while (True):
        bs = uart.read(1)
        if ((bs == None) or (len(bs) == 0)):
            ## If configured, check CTS, if it is down toggle RTS for 250ms
            if ( (DEFAULT_CHECK_CTS == 0) or (uart.getCTS() == True) ):
                continue

            uart.setRTS(False)
            # print ("*******************CTS dropped, dropping RTS for 250ms")
            time.sleep(.250)
            uart.setRTS(True)
            continue

        # First byte must be 0x19 - drop all data till we find it
        if (offset == 0) and (bs[0] != 0x19):
            continue

        ba[offset] = bs[0]
        offset += 1
        if (offset == 5):
            msg_len = (ba[4] << 8) + ba[3] + 5

        if (offset == msg_len):
            message = bytes(ba[0:msg_len])
            break

    # Check for protocol trace messages
    if ((message[1] == 0x03) and (message[2] == 0x00)):
        type = message[5] & 0xFF;
        spyType = 0;

        if (type == 2):
            spyType = SPY_TRACE_TYPE_ACL_RX
        elif (type == 3):
            spyType = SPY_TRACE_TYPE_ACL_TX;
        elif (type == 1):
            spyType = SPY_TRACE_TYPE_HCI_COMMAND;
        elif (type == 0):
            spyType = SPY_TRACE_TYPE_HCI_EVENT;
        elif (type == 8):
            spyType = SPY_TRACE_TYPE_LMP_RECV;
        elif (type == 9):
            spyType = SPY_TRACE_TYPE_LMP_XMIT;
        else:
            spType = 0

        ForwardHciTraceToSpy (spyType, message[6:], scriptInstance);
        return (None)

    ## Intercept debug trace messages
    if ((message[1] == 0x02) and (message[2] == 0x00)):
        ForwardDebugMsgToSpy (message[5:], scriptInstance)
        return (None)

    return message

def hexdump(a):
    return " ".join(format(x, '02X') for x in a)

def receive_thread():
    global evtQ
    global bUseUart

    while True:
        if (bUseUart):
            evt = receive_from_uart();
        else:
            evt = receive_from_socket();

        if (evt == None):
            continue

        if (len(evt) == 0):
            continue

        # print ("Received Len: " + repr(len(evt)) + "  Data: " + hexdump(evt))

        if (evt[0] == 0x19 and evt[2] == HCI_CONTROL_GROUP_DEVICE):
            #print("receive_thread got event " + getEvtDescription (evt[1]))
            evtQ.put(evt)

def getWicedHciEvent():
    global evtQ

    if (evtQ.empty()):
        return (0, None)
    else:
        evt = evtQ.get()
        if (len(evt) > 5):
            return (evt[1], evt[5:])
        else:
            return (evt[1], None)

def disconnect_server():
    global sock
    sock.close()

def TraceToSpy (text, instance = -1):
    global scriptInstance

    tlen = len(text)
    bt_header = bytes([0, 0, (tlen & 0x00FF), (tlen >> 8), 0, 0, (instance & 0x00FF), 0])
    text_bytes = bytes (text, "utf-8")
    message = bt_header + text_bytes

    if (instance == -1):
        instance = scriptInstance

    udp_port = 9876 + instance
    udp_ip   = "127.0.0.1"

    usock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    usock.sendto (message, (udp_ip, udp_port))

def ForwardDebugMsgToSpy (msg, instance):
    tlen = len(msg)
    bt_header = bytes([0, 0, (tlen & 0x00FF), (tlen >> 8), 0, 0, (instance & 0x00FF), 0])
    message = bt_header + msg

    udp_port = 9876 + instance
    udp_ip   = "127.0.0.1"

    usock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    usock.sendto (message, (udp_ip, udp_port))

def ForwardHciTraceToSpy (type, data, instance):
    tlen  = len(data)
    bt_header = bytes([type, 0, (tlen & 0x00FF), (tlen >> 8), 0, 0, (instance & 0x00FF), 0])
    message = bt_header + data

    udp_port = 9876 + instance
    udp_ip   = "127.0.0.1"

    usock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    usock.sendto (message, (udp_ip, udp_port))
