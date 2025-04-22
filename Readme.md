# Dynamixel 
*  모든 joint는 4095단계 But 영점이 각 Joint마다 달러서 주의요망
* 영점 좌표: [4127, 3188, 3546, 3147, 1543, 489]
* 영점이동 기능 추가완료!
# 뉴로(빵빵이)
* 특이점 주의! 여기도 영점이 각 Joint마다 다름!
* 영점 좌표: [0,-15, -90, 0, -75, 38]
* Joint 1 -90~90
* Joint 2 -55~0
* Joint 3 0~-125
* Joint 4 -90~90
* Joint 5 -75~90
* Joint 6 = 38(고정)
# 변환연산
* 안전공간내에서만 움직일 수 있도록!
* ![[KakaoTalk_20250318_212600709.jpg]]
# 참고한 자료
* https://emanual.robotis.com/docs/kr/dxl/x/xm430-w350/#torque-enable
* http://docs.neuromeka.com/3.2.0/kr/IndyAPI/indydcp3_python/#stop_motionstop_category
* https://swimminglab.tistory.com/m/116
* https://amiandappi.tistory.com/82
## 에러사항
* 홈 위치가 75 -> 4127로 좌표가 바뀌는 문제가 있음!
	* Torque가 해제되면 좌표계가 바뀜!![[Pasted image 20250318143011.png]]
	* Torque PWM 파워를 1로 설정하고 Present location 좌표계를 유지하여 해결 완료
* 뉴로메카의 좌표 실시간 업데이트가 늦음(1초?)
	* 차라리 업데이트간 움직이는 시간를 늘려서 움직임을 크고 부드럽게 구현 + BleedingType.OVERRIDE + 뉴로메카의 이동속도 30%로 설정

# 코드
* Dynamixel 제어
```python
import os
from dynamixel_sdk import *

from threading import Thread
import sys
import time

class DynamixelControl():
    def __init__(self):
        if os.name == 'nt':
            import msvcrt
            def getch():
                return msvcrt.getch().decode()

        else:
            import sys, tty, termios
            self.fd = sys.stdin.fileno()
            self.old_settings = termios.tcgetattr(self.fd)

        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_PWM                    = 36
        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_GOAL_POSITION          = 116
        self.LEN_GOAL_POSITION           = 4         # Data Byte Length
        self.ADDR_PRESENT_POSITION       = 132
        self.LEN_PRESENT_POSITION        = 4         # Data Byte Length
        self.BAUDRATE                    = 1000000

        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        # Initialize PortHandler instance
        self.portHandler = PortHandler('/dev/ttyUSB0')

        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(2.0)

        # Initialize GroupBulkWrite instance
        self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)

        # Initialize GroupBulkRead instace for Present Position
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)

        # Open port
        if self.portHandler.openPort():
            # print("Succeeded to open the port")
            pass

        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            self.getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            # print("Succeeded to change the baudrate")
            pass

        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            self.getch()
            quit()

    def getch(self):
        try:
            tty.setraw(sys.stdin.fileno())
            self.ch = sys.stdin.read(1)

        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

        return self.ch

    def engage_torque(self, motor_id, torque_status):
        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,motor_id, self.ADDR_TORQUE_ENABLE, torque_status)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        else:
            print("Dynamixel#%d has been successfully torqued" % motor_id)

    def current_position(self, motor_id):
        # Add parameter storage for Dynamixel#1 present position
        motor_addparam_result = self.groupBulkRead.addParam(motor_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        if motor_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % motor_id)
            quit()

            # Bulkread present position and LED status
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupbulkread data of Dynamixel#1 is available
        dxl_getdata_result = self.groupBulkRead.isAvailable(motor_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % motor_id)
            quit()

        # Get present position value
        motor_present_position = self.groupBulkRead.getData(motor_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
    
        return(motor_present_position)

    def move_to_goal(self, motor_id, goal):

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)), DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal))]

        # Add Dynamixel#1 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = self.groupBulkWrite.addParam(motor_id, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % motor_id)
            quit()

        # Bulkwrite goal position and LED value
        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        self.groupBulkWrite.clearParam()

    def move_to_home(self):
        self.move_to_goal(0, 2020)
        self.move_to_goal(1, 3430)
        self.move_to_goal(2, 3600)
        self.move_to_goal(3, 3120)
        self.move_to_goal(4, 1521)
        self.move_to_goal(5, 600)
        self.move_to_goal(6, 3060)

    def position(self):
        axisin=7
        array = [0,0,0,0,0,0,0]

        # Added to simulate Neuromeka
        # Comment it when simulate Zeus
        self.engage_torque(2, 1)

        for i in range(axisin):
                array[i]=self.current_position(i)

        # print(array)
        return array

    def engage_pwm(self, motor_id, power, axis):
        for i in range(axis):
            self.engage_torque(i, 0)

        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler,motor_id, self.ADDR_PWM, power)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        else:
            print("Dynamixel#%d has been successfully modulated" % motor_id)

        for i in range(axis):
            self.engage_torque(i, 1)
    
def thread_position():

    axisin=7
    array = [0,0,0,0,0,0,0]
    tdc = DynamixelControl()

    while True:
        tdc = DynamixelControl()
        for i in range(axisin):
            array[i]=tdc.current_position(i)

        print(array)
        time.sleep(1)
    
    return array

def main():
    # motor0, motor1, motor2, motor3, motor4, motor5 = 0,1,2,3,4,5,6
    dc = DynamixelControl()
    axis = 7

    th1 = Thread(target=thread_position,)
    th1.start()

    ans = input("Q to torque, W to home, E to store, R to quit")
    if ans == "q":
        for i in range(axis):
            dc.engage_torque(i, 1)

        print("q")
            
    elif ans == "w":
        dc.home()
        print("w")

           
    elif ans == "e":
        for i in range(axis):
            dc.engage_torque(i, 0)

        print("e")

    elif ans == "r":
        dc.groupBulkRead.clearParam()
        dc.portHandler.closePort()
        print("goodbye")
        sys.exit(0)
            
    else:
        print("Incorrect input")

if __name__ == '__main__':
    while True:
        main()

```
* Indy 제어
```python
from neuromeka import IndyEye
from neuromeka.eye import *
from neuromeka import IndyDCP3
from neuromeka import BlendingType
from neuromeka import StopCategory

from threading import Thread
import time

class IndyEyeClass:
    def __init__(self):
        self.eye = IndyEye("192.168.214.20")

    def shot(self):
        # type= PNG
        # print(type(self.current_image))

        while True:
            # print("a")
            start = time.time()
            self.current_image = self.eye.image()
            print(time.time()-start)
            self.current_image.save("/home/sungwoo/workspace/bbang/indyeye-python/current_image.png")

class IndyMoveClass:
    def __init__(self):
        self.indy = IndyDCP3(robot_ip="192.168.214.20", index=0)
        
    def position(self):
        while True:
            joint_angle = self.indy.get_control_state()['q']
            workspace_angle = self.indy.get_control_state()['p']
            print("joint_angle is:", joint_angle)
            print("workspace_angle is:", workspace_angle)
            time.sleep(1)

    def position_lib_joint(self):
        joint_angle = self.indy.get_control_state()['q']
        new_joint_angle = [round(joint_angle[0],0),\
                           round(joint_angle[1],0),\
                           round(joint_angle[2],0),\
                           round(joint_angle[3],0),\
                           round(joint_angle[4],0),\
                           round(joint_angle[5],0)]
        return(new_joint_angle)
    
    def position_lib_eef(self):
        joint_angle = self.indy.get_control_state()['p']
        new_joint_angle = [round(joint_angle[0],0),\
                           round(joint_angle[1],0),\
                           round(joint_angle[2],0),\
                           round(joint_angle[3],0),\
                           round(joint_angle[4],0),\
                           round(joint_angle[5],0)]
        return(new_joint_angle)

    def home(self):
        self.indy.move_home()

    def teaching(self, status):
        self.indy.set_direct_teaching(enable=status)

    def waypoint_joint(self, array):
        ttarget0 = [211.2912, -54.37297, -53.52943, -3.7350252, 1.8712654, -0.015391737]
        ttarget1 = [194.84499, -56.14404, -53.699265, -0.16174921, 1.7547668, -0.015391737]
        ttarget2 = [179.61856, -57.7746, -52.574802, -1.7419313, 1.7569966, -0.015391737]
        ttarget3 = [163.393407, -59.312687, -50.672848, -5.5159526, 1.504148, -0.015391737]

        self.indy.add_joint_waypoint(array)
        self.indy.move_joint_waypoint(move_time=1)
        # print(self.indy.get_joint_waypoint())
        
        # self.indy.add_joint_waypoint(ttarget0)
        # self.indy.add_joint_waypoint(ttarget1)
        # self.indy.add_joint_waypoint(ttarget2)
        # self.indy.add_joint_waypoint(ttarget3)
        # self.indy.move_joint_waypoint()
        # print(self.indy.get_joint_waypoint())

        # self.indy.movej(jtarget=ttarget0)
        # time.sleep(3)
        # self.indy.movej(jtarget=ttarget1)
        # time.sleep(3)
        # self.indy.movej(jtarget=ttarget2)
        # time.sleep(3)
        # self.indy.movej(jtarget=ttarget3)
        # time.sleep(3)

    def movej(self, array):
        # self.indy.movej(jtarget=array, vel_ratio=100, acc_ratio=100, blending_type=BlendingType.OVERRIDE)
        self.indy.movej(jtarget=array, vel_ratio=100, acc_ratio=100, blending_type=BlendingType.OVERRIDE)

    def stop(self):
        self.indy.stop_motion(StopCategory.CAT2)

    def movel(self, array):
        # X,Y,Z,U,V,W
        # Reference from image
        self.indy.movel(ttarget=array, vel_ratio=100, acc_ratio=100, base_type=TankBaseType.ABSOLUTE)

def main():

    im = IndyMoveClass()

    th2 = Thread(target=IndyMoveClass().position,)
    th2.start()

    # im.teaching(True)
    # time.sleep(10)
    # im.teaching(False)

    im.home()

    while True:
        im.waypoint()
    

if __name__ == '__main__':
    main()
```
* TeleOps 미들웨어
```python
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

```
# 결과
* 후에 좀더 소프트웨어 개선할 것!
* ![[KakaoTalk_20250318_210619377.mp4]]
