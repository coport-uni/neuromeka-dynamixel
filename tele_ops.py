from dyna_controller import DynamixelControl
from indy_mover import IndyMoveClass

from threading import Thread
import numpy as np
import time

import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

dc_value = [0,0,0,0,0,0]
sleep_time = 1

def thread_position():
    global dc_value
    global sleep_time

    axisin=6
    array = [0,0,0,0,0,0]
    tdc = DynamixelControl()

    # Added to simulate Neuromeka
    # Comment it when simulate Zeus
    tdc.engage_torque(2, 1)

    while True:
        tdc = DynamixelControl()
        for i in range(axisin):
            array[i]=tdc.current_position(i)

        # print(array)
        dc_value = array

        time.sleep(sleep_time)
    
    # return array

def mapping_neuromeka(array):
    global dc_value
    global sleep_time

    # NeuroHome
    new_array = [0,-15,-90,0,-75,38]
    time.sleep(sleep_time)
    check_array = dc_value

    # Update filter!
    if array == check_array:
        new_array[0] = np.interp(array[0] + (12 * (3300-1040)/(90+90)), [1040,3300], [-90,90])
        new_array[1] = np.interp(array[1] + (9 * (3700-3000)/(0+55)), [3000,3700], [-55,0])
        new_array[2] = np.interp(array[3] + (-2 * (3500-2400)/(0+125)), [2400,3400], [0,-125])
        new_array[3] = np.interp(array[4] + (-2 * (2500-2000)/(90+90)), [500,2500], [-90,90])
        new_array[4] = np.interp(array[5] + (0 * (5700-3600)/(0+125)), [600,1900], [-75,90])
        new_array[5] = 38
    
    else:
        print("filter active")

    return new_array

def main():
    global dc_value
    global sleep_time

    axis= 6 
    im = IndyMoveClass()
    dc = DynamixelControl()

   # Engage toque and move to position
    for i in range(axis):
        dc.engage_pwm(i, 400, axis)

    dc.move_to_home()
    im.home()
    time.sleep(5)

    for i in range(axis):
        dc.engage_pwm(i, 1, axis)
    
    # Added to simulate Neuromeka
    # Comment it when simulate Zeus
    dc.engage_pwm(2,400,axis)

    th1 = Thread(target=thread_position,)
    th1.start()

    im.stop()

    while True:

        print("dc_value:", dc_value)
        im_value = mapping_neuromeka(dc_value)

        print("im_value:", im_value)
        time.sleep(sleep_time)

        im.movej(im_value)
        # im.waypoint(im_value)

if __name__ == '__main__':
    main()
