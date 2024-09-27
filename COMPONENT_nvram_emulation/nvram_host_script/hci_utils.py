import sys
import time
from wiced_hci_impl import *

def hexprint(a):
    return " ".join(format(x, '02X') for x in a)

def flushWicedEvents(num_secs):
    startTime = time.time()
    while (True):
        evt_code, evt_data = getWicedHciEvent();
        if (evt_code == 0):
            curTime = time.time()
            diff = (curTime - startTime)
            if (diff > num_secs):
                break;
            time.sleep(0.1)

def waitForEvents (wait_evt_code, num_events_expected, max_time_to_wait = 0):
    startTime = time.time()
    num_matches = 0

    while (num_matches < num_events_expected):
        evt_code, evt_data = getWicedHciEvent();

        if (evt_code == 0):
            if (max_time_to_wait != 0):
                elapsed = (time.time() - startTime)
                if (elapsed > max_time_to_wait):
                    print ("Timed out waiting for event")
                    return (num_matches)

            time.sleep (0.1)
            continue

        if (evt_code == wait_evt_code):
            num_matches = num_matches + 1
        else:
            print ("Waiting, got unexpected event")

    return (num_matches)

def getEvent (max_time_to_wait = 0):
    startTime = time.time()

    while (True):
        evt_code, evt_data = getWicedHciEvent();

        if (evt_code == 0):
            if (max_time_to_wait != 0):
                elapsed = (time.time() - startTime)
                if (elapsed > max_time_to_wait):
                    print ("Timed out waiting for any event")
                    return (0, None)

            time.sleep (0.1)
            continue

        return (evt_code, evt_data)
