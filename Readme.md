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

    def position(self):
        axisin=6
        array = [0,0,0,0,0,0]

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

    axisin=6
    array = [0,0,0,0,0,0]
    tdc = DynamixelControl()

    while True:
        tdc = DynamixelControl()
        for i in range(axisin):
            array[i]=tdc.current_position(i)

        print(array)
        time.sleep(1)
    
    return array

def main():
    # motor0, motor1, motor2, motor3, motor4, motor5 = 0,1,2,3,4,5
    dc = DynamixelControl()
    axis = 6

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

    def home(self):
        self.indy.move_home()

    def teaching(self, status):
        self.indy.set_direct_teaching(enable=status)

    def waypoint(self, array):
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
        self.indy.movej(jtarget=array, vel_ratio=100, acc_ratio=300, blending_type=BlendingType.OVERRIDE)

    def stop(self):
        self.indy.stop_motion(StopCategory.CAT2)
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

from threading import Thread
import numpy as np
import time

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

```
# 결과
* 후에 좀더 소프트웨어 개선할 것!
* ![[KakaoTalk_20250318_210619377.mp4]]
