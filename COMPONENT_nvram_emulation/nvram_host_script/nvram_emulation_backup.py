import sys
import time
from wiced_hci_impl import *
from hci_utils import *
import struct
import json
import base64
from struct import pack, unpack

HCI_CONTROL_GROUP_DEVICE = 0x00
HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA = (HCI_CONTROL_GROUP_DEVICE << 8) | 0x05
HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA = (HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x06
HCI_CONTROL_EVENT_NVRAM_DATA = (HCI_CONTROL_GROUP_DEVICE << 8 ) | 0x04

def main():
    # Open COM port with the provided baud rate and timeout
    InitWicedHCI()

    # open file if it was pased on command line
    nvram_filename = 'nvram.json'
    backup_info = {}
    if os.path.isfile(nvram_filename):
        with open(nvram_filename, 'r') as f:
            backup_info = json.load(f)
            f.close()
        for id in backup_info.keys():
            s = backup_info[str(id)]
            data = base64.b16decode(s)
            hci_nvram_push_cmd = pack("<H%dB" % len(data), int(id), *data)
            sendWicedHciCommand(HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA,hci_nvram_push_cmd)
            print("sent nvram backup restore")
    else:
        print("No nvram backup data to push")

    # now backup_info has been sent
    print("Monitoring for nvram backup events, Ctrl-C to quit")
    while(True):
        evt_code, evt_data = getEvent(5);
        if (evt_code == 0):
            print ("timeout waiting for response")
        if (evt_code == HCI_CONTROL_EVENT_NVRAM_DATA):
            # pull off first 2 bytes as vs_id, store to json as dictionary item
            id_tuple = struct.unpack('<H', evt_data[0:2])
            id_new = str(id_tuple[0])
            val = base64.b16encode(evt_data[2:]).decode("ascii")
            print("got nvram data update {}".format(id_new))
            backup_info = {}
            if os.path.isfile(nvram_filename):
                with open(nvram_filename) as f:
                    backup_info = json.load(f)
                    f.close()
            backup_info.update({id_new:val})
            with open(nvram_filename, "w") as f:
                json.dump(backup_info, f)
                f.close()
        else:
            print ("Got unexpected event: {}".format(evt_code))

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python nvram_emulation_backup.py <COM_port> <baud_rate>")
    else:
        main()
