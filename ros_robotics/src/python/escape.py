#!/usr/bin/env python3

from robot_control_class import RobotControl
import time
import math

class MoveRobot:
    def __init__(self):
        self.robotcontrol=RobotControl()

    def move_straight(self):
        self.robotcontrol.move_straight()
    def findmax(self,list):
        a = max(list)
        idx = list.index(a)
        print('gia tri:',a)
        print('chon:',idx)
        return idx

    def escape(self):
        m = self.robotcontrol.get_laser(15)
        n= self.robotcontrol.get_laser(360)
        p= self.robotcontrol.get_laser(719)
        while m<100 or n<100 or p<100:
            # quay
            ls=[m,n,p]
            idx = self.findmax(ls)
            self.robotcontrol.rotate((1-idx)*85)

            # can chinh
            h1=self.robotcontrol.get_laser(160)
            h2=self.robotcontrol.get_laser(180)
            print('h1',h1,' ','h2',h2)
            h3=self.robotcontrol.get_laser(560)
            h4=self.robotcontrol.get_laser(540)
            print('h3',h3,' ','h4',h4)
            al1 = (h2*math.cos(math.radians(45))-h1*math.cos(math.radians(40)))/(h2*math.sin(math.radians(45))-h1*math.sin(math.radians(40)))
            print('al1',al1)
            al2 = (h4*math.cos(math.radians(135))-h3*math.cos(math.radians(140)))/(h4*math.sin(math.radians(135))-h3*math.sin(math.radians(140)))
            print('al2',al2)
            g1 = math.degrees(math.atan(al1))
            g2 = math.degrees(math.atan(al2))
            ls2=[g2,0,g1]
            print('ls2:',ls2)
            print('goc:',ls2[idx])
            self.robotcontrol.rotate(ls2[idx])    

            #go
            a=self.robotcontrol.get_laser(360)
            c=self.robotcontrol.get_laser(15)
            while a>1.2 and c<30:
                self.move_straight()
                a=self.robotcontrol.get_laser(360)
               
            self.robotcontrol.stop_robot()
            m = self.robotcontrol.get_laser(0)
            n= self.robotcontrol.get_laser(360)
            p= self.robotcontrol.get_laser(719)
        self.robotcontrol.stop_robot()

        #Di ra
    """  ls=[m,n,p]
        idx = self.findmax(ls)
        self.robotcontrol.rotate((1-idx)*85)
        h1=self.robotcontrol.get_laser(160)
        h2=self.robotcontrol.get_laser(180)
        print('h1',h1,' ','h2',h2)
        h3=self.robotcontrol.get_laser(560)
        h4=self.robotcontrol.get_laser(540)
        print('h3',h3,' ','h4',h4)
        al1 = (h2*math.cos(math.radians(45))-h1*math.cos(math.radians(40)))/(h2*math.sin(math.radians(45))-h1*math.sin(math.radians(40)))
        print('al1',al1)
        al2 = (h4*math.cos(math.radians(135))-h3*math.cos(math.radians(140)))/(h4*math.sin(math.radians(135))-h3*math.sin(math.radians(140)))
        print('al2',al2)
        g1 = math.degrees(math.atan(al1))
        g2 = math.degrees(math.atan(al2))
        ls2=[g2,0,g1]
        print('ls2:',ls2)
        print('goc:',ls2[idx])
        self.robotcontrol.rotate(ls2[idx])
        b=self.robotcontrol.get_laser(0)
        while 0>math.inf:
            self.move_straight()
            b=self.robotcontrol.get_laser(0) 
"""
mr1 = MoveRobot()
mr1.escape()
