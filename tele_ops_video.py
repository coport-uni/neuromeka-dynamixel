from dyna_controller import DynamixelControl
from indy_mover import IndyMoveClass

from multiprocessing import Process
from threading import Thread
import numpy as np
import time
import cv2
import os

dc_value = [0,0,0,0,0,0]
sleep_time = 0.25
im_gvalue = [0,0,0,0,0,0]
internal_frame = cv2.imread("/home/sungwoo/workspace/bbang/teleops/Standby.jpg")

def capture_camera_ex():
    cap0 = cv2.VideoCapture(4)
    cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    count = 0
    ret = True

    while (ret):
        ret, frame = cap0.read()

        if (ret==False):
            break
        if(int(cap0.get(1)) % 2 == 0):
            pause_frame = 9
            if (count > pause_frame-1):
                # t1 = time.time()
                # new_frame = cv2.resize(frame,(640,480))
                # t2 = time.time()
                # print(t2-t1)
                cv2.imwrite("/home/sungwoo/workspace/bbang/teleops/test_dataset/external/" + str(count - pause_frame) + " " + str(im.position_lib_joint()) + ".jpg",frame)
            
            count +=1
        
    cap0.release()

def capture_camera_in():
    global internal_frame

    im = IndyMoveClass()

    cap1 = cv2.VideoCapture(6)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    count = 0
    ret = True

    while (ret):
        ret, frame = cap1.read()
        internal_frame = frame

        if (ret==False):
            break
        if(int(cap1.get(1)) % 2== 0):
            # new_frame = cv2.resize(frame,(640,480))
            # + str(im.position_lib())
            cv2.imwrite("/home/sungwoo/workspace/bbang/teleops/test_dataset/internal/" + str(count) + " " + str(im.position_lib_eef())+ ".jpg",frame)
            count +=1
        
        
    cap1.release()
    
def thread_position():
    global dc_value
    global sleep_time

    axisin=7
    array = [0,0,0,0,0,0,0]
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
    new_array = [0,-15,-90,0,-75,0]
    time.sleep(sleep_time)
    check_array = dc_value

    # Update filter later!
    if array == check_array:
        new_array[0] = round(np.interp(array[0] + (12 * (3300-1040)/(90+90)), [1040,3300], [-90,90]),0)
        new_array[1] = round(np.interp(array[1] + (15 * (3700-3000)/(0+75)), [3000,3700], [-75,0]),0)
        new_array[2] = round(np.interp(array[3] + (-16 * (3500-2400)/(0+145)), [2400,3400], [0,-145]),0)
        new_array[3] = round(np.interp(array[4] + (-22 * (2500-2000)/(90+100)), [500,2500], [-90,100]),0)
        new_array[4] = round(np.interp(array[5] + (0 * (5700-3600)/(90+75)), [600,1900], [-75,90]),0)
        new_array[5] = round(np.interp(array[6] + (-2 * (4000-2000)/(90+90)), [2000,4000], [90,-90]),0)
    else: 
        print("filter")

    return new_array

def main():
    global dc_value
    global im_gvalue
    global sleep_time
    global internal_frame

    im = IndyMoveClass()
    dc = DynamixelControl()

    axis= 6

    os.makedirs("test_dataset",exist_ok=True)
    os.makedirs("test_dataset/external",exist_ok=True)
    os.makedirs("test_dataset/internal",exist_ok=True)

   # Engage toque and move to position
    for i in range(axis):
        dc.engage_pwm(i, 400, axis)

    dc.move_to_home()
    im.home()
    time.sleep(3)

    for i in range(axis):
        dc.engage_pwm(i, 1, axis)
    
    # Added to simulate Neuromeka
    # Comment it when simulate Zeus
    dc.engage_pwm(2,400,axis)

    im.stop()

    th2 = Thread(target=capture_camera_ex,)
    th2.start()

    th3 = Thread(target=capture_camera_in,)
    th3.start()

    th1 = Thread(target=thread_position,)
    th1.start()

    while True:

        print("dc_value:", dc_value)
        im_value = mapping_neuromeka(dc_value)
        print("im_value:", im_value)
        # im.waypoint(im_value)
        im.movej(im_value)
        im_gvalue = im_value

        cv2.imshow("control",internal_frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    main()
