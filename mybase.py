#!/user/bin/env python3

import ev3dev2
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor, MediumMotor, SpeedRPS, SpeedPercent, MoveTank, MoveDifferential, MoveSteering
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, Sensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.wheel import Wheel, EV3Tire
from ev3dev2.power import PowerSupply
from ev3dev2.sound import Sound
from ev3dev2.port import LegoPort
import sys
import time
import math
import ev3fast


class myBase(Sound):

    def console(self, item):
        print(item, file=sys.stderr)

    def display_log(self, Log, time_axis):
        for i in range(len(Log[0])):
            for j in Log:
                print(j[i], file=sys.stderr)
            print("----------------------------------------40", file=sys.stderr)
        for t in time_axis:
            print(t, file=sys.stderr)
    
    def __init__(self, leftMotor, rightMotor, frontClawMotor, sorterMotor, tank, steerer, WHEEL_RADIUS_mm,
                 AXLE_TRACK_mm, leftCS, rightCS, lightArr, frontUltra, sideUltra, sorterCS):
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.sorterMotor = sorterMotor
        self.leftMotor.polarity = "inversed"
        self.rightMotor.polarity = "inversed"
        self.frontClawMotor = frontClawMotor
        self.tank = tank
        self.steerer = steerer
        self.WHEEL_RADIUS_mm = WHEEL_RADIUS_mm
        self.AXLE_TRACK_mm = AXLE_TRACK_mm

        self.leftCS = leftCS
        self.rightCS = rightCS
        self.lightArr = lightArr
        self.lightArr.mode = "CAL"
        self.frontUltra = frontUltra
        self.sideUltra = sideUltra
        self.sorterCS = sorterCS

        self.LrMax = 0
        self.LrMin = 0
        self.LgMax = 0
        self.LgMin = 0
        self.LbMax = 0
        self.LbMin = 0
        self.RrMax = 0
        self.RrMin = 0
        self.RgMax = 0
        self.RgMin = 0
        self.RbMax = 0
        self.RbMin = 0

        self.caliConstants = (
            (self.LrMax, self.LrMin),
            (self.LgMax, self.LgMin),
            (self.LbMax, self.LbMin),
            (self.RrMax, self.RrMin),
            (self.RgMax, self.RgMin),
            (self.RbMax, self.RbMin),
        )

        self.Lr_FullBlackThreshold = 20
        self.Rr_FullBlackThreshold = 20
        self.L_greenSqr_rgDiff = 50
        self.R_greenSqr_rgDiff = 50 # subject to change

        self.RA_jump_threshold = 60
        self.RA_lightArr_threshold = 4 * 15

        Sound.__init__(self)
        self.beep()
        self.console("battery Voltage: {}mV".format(battery.measured_voltage * 0.001))
        self.console("\n")

        self.maxRot = 2.915

        self.gearBox = (
            (50,  # speed 0 (10mm/s)
             0.015,  # Kp
             0.0,  # Ki
             0.0  # Kd
             ), (
                75,  # speed 1 (100mm/s)
                0.018,  # Kp
                0.0,  # Ki
                0.0  # Kd
            )
        )

    def calibrate_LRCS(self):
        """Beep then immediately start collect white max. after collected max, beep twice. then wait a bit before the next session.
        Do the above for 5 times.
        Beep 3 times after white max collected.
        Wait a bit, then beep, then collect black min. after collected min, beep twice."""

        white_maxLRrgb_list = [[], [], [], [], [], []]
        for repeat in range(5):
            self.beep()
            self.steerer.on(0, SpeedPercent(10))
            timeStart = time.time()
            rawVals_list = [[], [], [], [], [], []]
            while time.time() - timeStart <= 2.5:
                Lrgb, Rrgb = self.leftCS.raw, self.rightCS.raw
                rawVals_list[0].append(Lrgb[0])
                rawVals_list[1].append(Lrgb[1])
                rawVals_list[2].append(Lrgb[2])
                rawVals_list[3].append(Rrgb[0])
                rawVals_list[4].append(Rrgb[1])
                rawVals_list[5].append(Rrgb[2])
            self.steerer.off()
            for i in range(len(white_maxLRrgb_list)):
                white_maxLRrgb_list[i].append(max(rawVals_list[i]))
            self.console(rawVals_list[i])
            self.beep()
            time.sleep(0.01)
            self.beep()
            time.sleep(5)

        self.beep()
        time.sleep(0.01)
        self.beep()
        time.sleep(0.01)
        self.beep()

        time.sleep(5)

        black_minLRrgb_list = [[], [], [], [], [], []]
        for repeat in range(5):
            self.beep()
            self.steerer.on(0, SpeedPercent(10))
            timeStart = time.time()
            rawVals_list = [[], [], [], [], [], []]
            while time.time() - timeStart <= 2.5:
                Lrgb, Rrgb = self.leftCS.raw, self.rightCS.raw
                rawVals_list[0].append(Lrgb[0])
                rawVals_list[1].append(Lrgb[1])
                rawVals_list[2].append(Lrgb[2])
                rawVals_list[3].append(Rrgb[0])
                rawVals_list[4].append(Rrgb[1])
                rawVals_list[5].append(Rrgb[2])
            self.steerer.off()
            for i in range(len(black_minLRrgb_list)):
                black_minLRrgb_list[i].append(min(rawVals_list[i]))
            self.console(rawVals_list[i])
            self.beep()
            time.sleep(0.01)
            self.beep()
            time.sleep(5)

        calibrated_min = []
        for i in range(len(black_minLRrgb_list)):
            minCurrVal = min(black_minLRrgb_list[i])
            if minCurrVal >= 5:
                calibrated_min.append(minCurrVal - 5) # why
            else:
                calibrated_min.append(0)
        calibrated_max = []
        for i in range(len(white_maxLRrgb_list)):
            calibrated_max.append(
                (max(white_maxLRrgb_list[i]) - calibrated_min[i]) / 0.98 + calibrated_min[i]
            ) # why

        self.console("""
        self.LrMax = {}
        self.LrMin = {}
        self.LgMax = {}
        self.LgMin = {}
        self.LbMax = {}
        self.LbMin = {}
        self.RrMax = {}
        self.RrMin = {}
        self.RgMax = {}
        self.RgMin = {}
        self.RbMax = {}
        self.RbMin = {}
        """.format(calibrated_max[0], calibrated_min[0], calibrated_max[1], calibrated_min[1], calibrated_max[2],
                   calibrated_min[2], calibrated_max[3], calibrated_min[3], calibrated_max[4], calibrated_min[4],
                   calibrated_max[5], calibrated_min[5]))

    def get_LRrgb_percent(self):
        """Returns a tuple containing 2 tuples: ( (Lrgb), (Lrgb) ), each containing the calibrated rgb percentage values."""

        raws = self.leftCS.raw + self.rightCS.raw
        calibrated = [(raws[i] - self.caliConstants[i][1]) / (self.caliConstants[i][0] - self.caliConstants[i][1]) * 100
                      for i in range(6)]
        # Ltoreturn = ( (Lrgb[0]-self.LrMin)/(self.LrMax-self.LrMin)*255, (Lrgb[1]-self.LgMin)/(self.LgMax-self.LgMin)*255, (Lrgb[2]-self.LbMin)/(self.LbMax-self.LbMin)*255 )
        # Rtoreturn = ( (Rrgb[0]-self.RrMin)/(self.RrMax-self.RrMin)*255, (Rrgb[1]-self.RgMin)/(self.RgMax-self.RgMin)*255, (Rrgb[2]-self.RbMin)/(self.RbMax-self.RbMin)*255 )
        return calibrated[0:3], calibrated[3:]

    def get_sorterCS_reading(self):  # incomplete
        # Since sorterCS connected to multiplexer and cannot read rgb tuple from multiplexer, i can only read
        # reflected red light.
        return self.sorterCS.value(0)

    def get_sensor_loopTime(self):
        startTime = time.time()
        readings = []
        time_axis = []
        while True:
            currTime = time.time() - startTime
            lightArr = self.get_lightArr_percent()
            readings.append(lightArr)
            # Lrgb,Rrgb = self.get_LRrgb_percent_old()
            # readings.append((Lrgb,Rrgb))
            time_axis.append(currTime)
            if currTime >= 15:
                break
            # time.sleep(0.05)
        for i in range(len(time_axis)):
            self.console((time_axis[i], readings[i]))
        self.console(sum([time_axis[i + 1] - time_axis[i] for i in range(len(time_axis) - 1)]) / len(time_axis))

    def move_frontClaw(self, direction):
        if direction == "up":
            # self.frontClawMotor.on_for_rotations(SpeedPercent(-25),1.35)
            self.frontClawMotor.on_for_rotations(SpeedPercent(-25), 1)
            time.sleep(0.5)
            self.console("in up, self.frontClawMotor.is_stalled: {}".format(self.frontClawMotor.is_stalled))
            while not self.frontClawMotor.is_stalled:
                self.frontClawMotor.on(SpeedPercent(-25))
        elif direction == "down":
            self.frontClawMotor.on_for_rotations(SpeedPercent(25), 1)
            time.sleep(0.5)
            self.console("in down, self.frontClawMotor.is_stalled: {}".format(self.frontClawMotor.is_stalled))
            while not self.frontClawMotor.is_stalled:
                self.frontClawMotor.on(SpeedPercent(25))
        else:
            self.console("error in frontClaw")