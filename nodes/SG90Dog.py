
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
        self.pwm = PWM(self.bus, self.i2c_address)
        self.pwm.setFreq(self.fPWM)

        self.setup()

        self.zeroBot()

        self.frLeg = Leg3d(side=1)
        self.flLeg = Leg3d(side=2)
        self.rlLeg = Leg3d(side=2)
        self.rrLeg = Leg3d(side=1)
        self.t = 0

        # walker = Walk3d(stride_height=-0.01,stride_length =.02)

        bpm = 138#music tempo
        self.freq = bpm/60.*2*pi
        self.amp = 0.015

    def setup(self):
        self.pwm = PWM(self.bus, self.i2c_address)
        self.pwm.setFreq(self.fPWM)

    def zeroBot(self):
        duty = self.a/180*90+self.b
        for ch in range(0,12):
            self.pwm.setDuty(ch,duty)

    def setLeg3d(self,tibia,femur,hip,tch,fch,hipch):
        # TODO: update servo adjustments
        if hipch == 9:
            fduty = self.a/180 * femur + self.b
            tduty = self.a/180 * tibia + self.b
            hduty = self.a/180 * (hip - 3) + self.b
        elif fch == 7:
            fduty = self.a/180 * (femur - 8) + self.b
            tduty = self.a/180 * (tibia - 7) + self.b
            hduty = self.a/180 * hip + self.b
        else:
            fduty = self.a/180 * femur + self.b
            tduty = self.a/180*tibia+self.b
            hduty = self.a/180*hip+self.b
        # print fduty,tduty,hduty
        self.pwm.setDuty(fch,fduty)
        self.pwm.setDuty(tch,tduty)
        self.pwm.setDuty(hipch,hduty)

    def senseForce(self, zFootRaw):
    	# Check to see if reading is greater than noise/offsets
    	if zFootRaw > 70:
    		# convert raw data into distance (meters)
    		float_to_m = 0.000007
    		dzFoot = float_to_m * zFootRaw
    		# Calculate force experience by foot
    		kFoot = 2795 # spring constant of foot, found empirically (N/m)
    		Ffoot = kFoot * dzFoot
    		# Calculate desired displacement using virtual spring (m)
    		kVirtual = 200 # spring constant of knee joint, guessed (N/m)
    		dz = Ffoot / kVirtual
    #		print dz
    		return dz
    	else:
    		# If not, simply return 0 and don't adjust z
    		return 0

    def doTurn(self,freq,yamp,zamp,t):
        phifr = 0
        phirl = 3*pi/2
        phifl = pi
        phirr = 1*pi/2
        yfl = -yamp*sin(freq*t+phifl)
        xfl = 0
        zfl = zamp*cos(freq*t+phifl)
        yfr = yamp*sin(freq*t+phifr)
        xfr = 0
        zfr = zamp*cos(freq*t+phifr)
        yrl = -yamp*sin(freq*t+phirl)
        xrl = 0
        zrl = zamp*cos(freq*t+phirl)
        yrr = yamp*sin(freq*t+phirr)
        xrr = 0
        zrr = zamp*cos(freq*t+phirr)
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr

    def doWalk(self,freq,xamp,zamp,t):
        phifr = 0
        phirl = pi/2
        phifl = pi
        phirr = 3*pi/2
        xfl = xamp*sin(freq*t+phifl)
        yfl = 0
        zfl = zamp*cos(freq*t+phifl)
        xfr = xamp*sin(freq*t+phifr)
        yfr = 0
        zfr = zamp*cos(freq*t+phifr)
        xrl = xamp*sin(freq*t+phirl)
        yrl = 0
        zrl = zamp*cos(freq*t+phirl)
        xrr = xamp*sin(freq*t+phirr)
        yrr = 0
        zrr = zamp*cos(freq*t+phirr)
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr

    def doStand(self,freq,amp,amp2,t):
        xfl = 0#amp*sin(freq*t)
        yfl = 0
        zfl = 0
        xfr = .00#amp*sin(freq*t)
        yfr = 0
        zfr = 0
        xrl = 0#amp*sin(freq*t)
        yrl = 0
        zrl = 0
        xrr = 0#amp*sin(freq*t)
        yrr = 0
        zrr = 0
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr

    def doDown(self,freq,amp,amp2,t):
        xfl = .01#amp*sin(freq*t)
        yfl = 0
        zfl = .025 #-.025
        xfr = .01#amp*sin(freq*t)
        yfr = 0
        zfr = .025 #-.025
        xrl = -.025#amp*sin(freq*t)
        yrl = 0
        zrl = .025 #-.025
        xrr = -.025#amp*sin(freq*t)
        yrr = 0
        zrr = .025 #-.025
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr

    def doSit(self,freq,amp,amp2,t):
        xfl = .01#amp*sin(freq*t)
        yfl = 0
        zfl = -0.02#.03
        xfr = .01#amp*sin(freq*t)
        yfr = 0
        zfr = -0.02#.03
        xrl = -0.01#-.025#amp*sin(freq*t)
        yrl = 0
        zrl = 0.02#-.03
        xrr = -0.01#-.025#amp*sin(freq*t)
        yrr = 0
        zrr = 0.02#-.03
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr


    def doSway(self,freq,amp,amp2,t):
        xfl = 0#amp*sin(freq*t)
        yfl = -amp*sin(.5*freq*t)
        zfl = 0
        xfr = 0#amp*sin(freq*t)
        yfr = amp*sin(.5*freq*t)
        zfr = 0
        xrl = 0#amp*sin(freq*t)
        yrl = amp*sin(.5*freq*t)
        zrl = 0
        xrr = 0#amp*sin(freq*t)
        yrr = -amp*sin(.5*freq*t)
        zrr = 0
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr

    def doBump(self,freq,amp,amp2,t):
        xfl = 0#amp*sin(freq*t)
        yfl = 0
        zfl = amp*sin(freq*t)
        xfr = 0#amp*sin(freq*t)
        yfr = 0
        zfr = amp*sin(freq*t)
        xrl = 0#amp*sin(freq*t)
        yrl = 0
        zrl = amp*sin(freq*t)
        xrr = 0#amp*sin(freq*t)
        yrr = 0
        zrr = amp*sin(freq*t)
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr

    def doStompL(self,freq,amp,amp2,t):
        xfl = 0#amp*sin(freq*t)
        yfl = amp*cos(.5*freq*t)
        zfl = amp*sin(freq*t)
        xfr = 0#amp*sin(freq*t)
        yfr = -2*amp
        zfr = amp
        xrl = -amp#amp*sin(freq*t)
        yrl = -2*amp
        zrl = 0
        xrr = -amp#amp*sin(freq*t)
        yrr = 2*amp
        zrr = 0
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr

    def doHighFiveL(self,freq,amp,amp2,t):
        xfl = 0.01 #-0.05
        yfl = 0
        zfl = 0.01 #-0.015 #-.0254*3.582
        xfr = 0
        yfr = 0
        zfr = 0
        xrl = 0
        yrl = 0
        zrl = 0
        xrr = 0
        yrr = 0
        zrr = 0
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr

    def doHighFiveR(self,freq,amp,amp2,t):
        xfl = 0
        yfl = 0
        zfl = 0
        xfr = 0.07 #-0.0785
        yfr = 0
        zfr = 0.09 # divide by 0 error (atan(y/z) in Leg3d -> -.0254*3.582
        xrl = 0
        yrl = 0
        zrl = 0
        xrr = 0
        yrr = 0
        zrr = 0
        return xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr

    def update(self,dt,action,freq,amp,force1,force2,force3,force4):
        self.t+=dt
        self.amp = amp
        self.freq = freq
        t = self.t
#        print force1
        if action=="stompleft":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doStompL(freq,amp,amp,t)
        elif action== "bump":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doBump(freq,amp,amp,t)
        elif action=="sway":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doSway(freq,amp,amp,t)
        elif action=="sit":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doSit(freq,amp,amp,t)
        elif action=="down":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doDown(freq,amp,amp,t)
        elif action=="stand":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doStand(freq,amp,amp,t)
        elif action=="walk":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doWalk(freq,amp,amp,t)
        elif action=="turn":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doTurn(freq,amp,amp,t)
        elif action=="highfiveleft":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doHighFiveL(freq,amp,amp,t)
        elif action=="highfiveright":
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = self.doHighFiveR(freq,amp,amp,t)
        else:
            xfl,yfl,zfl,xfr,yfr,zfr,xrl,yrl,zrl,xrr,yrr,zrr = 0,0,0,0,0,0,0,0,0,0,0,0

        # Adjust z position using force sensors
        dz1 = self.senseForce(force1)
        zfr_actual = zfr + dz1


        dz2 = self.senseForce(force2)
        zfl_actual = zfl + dz2

        # zfr_actual = zfr + dz1 # Temporarily reference second sensor due to spiking issue

        dz3 = self.senseForce(force3)
        zrl_actual = zrl + dz3

        dz4 = self.senseForce(force4)
        zrr_actual = zrr + dz4

        print(zfr_actual, zfl_actual, zrl_actual, zrr_actual)

#        flfem,fltib,flhip = self.flLeg.servoAngles(xfl,yfl,zfl)
#        frfem,frtib,frhip = self.frLeg.servoAngles(xfr,yfr,zfr)
#        rlfem,rltib,rlhip = self.rlLeg.servoAngles(xrl,yrl,zrl)
#        rrfem,rrtib,rrhip = self.rrLeg.servoAngles(xrr,yrr,zrr)

        # Update z parameters and calculate angles
        flfem,fltib,flhip = self.flLeg.servoAngles(xfl,yfl,zfl_actual)
        frfem,frtib,frhip = self.frLeg.servoAngles(xfr,yfr,zfr_actual)
        rlfem,rltib,rlhip = self.rlLeg.servoAngles(xrl,yrl,zrl_actual)
        rrfem,rrtib,rrhip = self.rrLeg.servoAngles(xrr,yrr,zrr_actual)

        # Pass angles to setLeg3d to send servos PWM commands
        self.setLeg3d(frfem,frtib,frhip,0,1,2)
        self.setLeg3d(flfem,fltib,flhip,3,4,5)
        self.setLeg3d(rlfem,rltib,rlhip,6,7,8)
        self.setLeg3d(rrfem,rrtib,rrhip,9,10,11)
