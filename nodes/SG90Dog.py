
#import keyboard
from smbus import SMBus
from PCA9685 import PWM
import time
from math import sin
from Leg3d import Leg3d
from numpy import *


class SG90Dog:

    def __init__(self):
        self.state = 2
        self.fPWM = 50
        self.i2c_address = 0x40 # (standard) adapt to your module
        self.channel = 0 # adapt to your wiring
        self.a = 8.5 # adapt to your servo
        self.b = 2  # adapt to your servo

        self.bus = SMBus(3) # Raspberry Pi revision 2
        self.pwm = PWM(bus, i2c_address)
        self.pwm.setFreq(fPWM)

        self.setup()

        self.zeroBot()

        self.frLeg = Leg3d(side=1)
        self.flLeg = Leg3d(side=2)
        self.lrLeg = Leg3d(side=2)
        self.rrLeg = Leg3d(side=1)
        self.t = 0

        # walker = Walk3d(stride_height=-0.01,stride_length =.02)

        bpm = 138#music tempo
        self.freq = bpm/60.*2*pi
        self.amp = 0.015

    def setup(self):
        self.pwm = PWM(self.bus, self.i2c_address)
        self.pwm.setFreq(self.fPWM)

    def zeroBot(self.):
        duty = a/180*90+b
        for ch in range(0,16):
            self.pwm.setDuty(ch,duty)

    def setHips(self,angle):
        duty = self.a / 180 * angle + self.b
        for ch in range(8,12):
            self.pwm.setDuty(ch,duty)

    def setLeg(self,femur,tibia,ch):
        fduty = self.a / 180 * femur + self.b
        tduty = self.a/180*tibia+self.b
        self.pwm.setDuty(ch,fduty)
        self.pwm.setDuty(ch+1,tduty)

    def setLeg3d(self,femur,tibia,hip,ch,hipch):
        fduty = self.a/180 * femur + self.b
        tduty = self.a/180*tibia+self.b
        hduty = self.a/180*hip+self.b
        # print fduty,tduty,hduty
        self.pwm.setDuty(ch,fduty)
        self.pwm.setDuty(ch+1,tduty)
        self.pwm.setDuty(hipch,hduty)


    def doTurn(freq,yamp,zamp,t):
        phifr = 0
        philr = 3*pi/2
        phifl = pi
        phirr = 1*pi/2
        yfl = -yamp*sin(freq*t+phifl)
        xfl = 0
        zfl = zamp*cos(freq*t+phifl)
        yfr = yamp*sin(freq*t+phifr)
        xfr = 0
        zfr = zamp*cos(freq*t+phifr)
        ylr = -yamp*sin(freq*t+philr)
        xlr = 0
        zlr = zamp*cos(freq*t+philr)
        yrr = yamp*sin(freq*t+phirr)
        xrr = 0
        zrr = zamp*cos(freq*t+phirr)
        return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr

    def doWalk(freq,xamp,zamp,t):
        phifr = 0
        philr = pi/2
        phifl = pi
        phirr = 3*pi/2
        xfl = xamp*sin(freq*t+phifl)
        yfl = 0
        zfl = zamp*cos(freq*t+phifl)
        xfr = xamp*sin(freq*t+phifr)
        yfr = 0
        zfr = zamp*cos(freq*t+phifr)
        xlr = xamp*sin(freq*t+philr)
        ylr = 0
        zlr = zamp*cos(freq*t+philr)
        xrr = xamp*sin(freq*t+phirr)
        yrr = 0
        zrr = zamp*cos(freq*t+phirr)
        return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr

    def doStand(freq,amp,t):
        xfl = 0#amp*sin(freq*t)
        yfl = 0
        zfl = 0
        xfr = .00#amp*sin(freq*t)
        yfr = 0
        zfr = 0
        xlr = 0#amp*sin(freq*t)
        ylr = 0
        zlr = 0
        xrr = 0#amp*sin(freq*t)
        yrr = 0
        zrr = 0
        return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr

    def doDown(freq,amp,t):
        xfl = .01#amp*sin(freq*t)
        yfl = 0
        zfl = -.025
        xfr = .01#amp*sin(freq*t)
        yfr = 0
        zfr = -.025
        xlr = -.025#amp*sin(freq*t)
        ylr = 0
        zlr = -.025
        xrr = -.025#amp*sin(freq*t)
        yrr = 0
        zrr = -.025
        return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr

    def doSit(freq,amp,t):
        xfl = .01#amp*sin(freq*t)
        yfl = 0
        zfl = .03
        xfr = .01#amp*sin(freq*t)
        yfr = 0
        zfr = .03
        xlr = -.025#amp*sin(freq*t)
        ylr = 0
        zlr = -.03
        xrr = -.025#amp*sin(freq*t)
        yrr = 0
        zrr = -.03
        return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr


    def doSway(freq,amp,t):
        xfl = 0#amp*sin(freq*t)
        yfl = -amp*sin(.5*freq*t)
        zfl = 0
        xfr = 0#amp*sin(freq*t)
        yfr = amp*sin(.5*freq*t)
        zfr = 0
        xlr = 0#amp*sin(freq*t)
        ylr = amp*sin(.5*freq*t)
        zlr = 0
        xrr = 0#amp*sin(freq*t)
        yrr = -amp*sin(.5*freq*t)
        zrr = 0
        return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr

    def doBump(freq,amp,t):
        xfl = 0#amp*sin(freq*t)
        yfl = 0
        zfl = amp*sin(freq*t)
        xfr = 0#amp*sin(freq*t)
        yfr = 0
        zfr = amp*sin(freq*t)
        xlr = 0#amp*sin(freq*t)
        ylr = 0
        zlr = amp*sin(freq*t)
        xrr = 0#amp*sin(freq*t)
        yrr = 0
        zrr = amp*sin(freq*t)
        return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr

    def doStompL(freq,amp,t):
        xfl = 0#amp*sin(freq*t)
        yfl = amp*cos(.5*freq*t)
        zfl = amp*sin(freq*t)
        xfr = 0#amp*sin(freq*t)
        yfr = -2*amp
        zfr = amp
        xlr = -amp#amp*sin(freq*t)
        ylr = -2*amp
        zlr = 0
        xrr = -amp#amp*sin(freq*t)
        yrr = 2*amp
        zrr = 0
        return xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr


    def update(self,dt,action,freq,amp):
        self.t+=dt
        self.amp = amp
        self.freq = freq
        if action=="stompleft":
            xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = self.doStompL(freq,amp,t)
        elif action== "bump":
            xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = self.doBump(freq,amp,t)
        elif action=="sway"
            xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = self.doSway(freq,amp,t)
        elif action=="sit":
            xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = self.doSit(freq,amp,t)
        elif action=="down":
            xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = self.doDown(freq,amp,t)
        elif action=="stand":
            xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = self.doStand(freq,amp,t)
        elif action=="walk":
            xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = self.doWalk(freq,amp,t)
        elif action=="turn":
            xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = self.doTurn(freq,amp,t)
        else:
            xfl,yfl,zfl,xfr,yfr,zfr,xlr,ylr,zlr,xrr,yrr,zrr = 0,0,0,0,0,0,0,0,0,0,0,0

        flfem,fltib,flhip = self.flLeg.servoAngles(xfl,yfl,zfl)
        frfem,frtib,frhip = self.frLeg.servoAngles(xfr,yfr,zfr)
        lrfem,lrtib,lrhip = self.lrLeg.servoAngles(xlr,ylr,zlr)
        rrfem,rrtib,rrhip = self.rrLeg.servoAngles(xrr,yrr,zrr)

        self.setLeg3d(frfem,frtib,frhip,0,8)
        self.setLeg3d(flfem,fltib,flhip,2,9)
        self.setLeg3d(lrfem,lrtib,lrhip,4,10)
        self.setLeg3d(rrfem,rrtib,rrhip,6,11)

