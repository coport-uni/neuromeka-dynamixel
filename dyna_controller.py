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
