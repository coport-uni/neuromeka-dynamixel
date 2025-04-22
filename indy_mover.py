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