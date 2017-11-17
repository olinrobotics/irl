#!/usr/bin/python
"""
  Control of UR5 from Universal Robots
    usage:
         ./ur5.py <cmd> [<log file>]
"""

# http://www.zacobria.com/universal-robots-zacobria-forum-hints-tips-how-to/script-via-socket-connection/

import sys
import time
import socket
import math
import struct
import datetime

UR5_HOST = "10.42.0.175" # TODO
UR5_PORT = 30003


# TODO:
# - verify units (degrees/rads and mm/meters?)
# - logging
# - extra thread?

HAND_ANGLES0 = 0.38359949145168565, 1.6393995499771168, 0.8566860014985311
HAND_ANGLES = 0.3566111421965905, 1.6135099949273701, 0.8512607535360434
HAND_ANGLES_STR = "%f, %f, %f" % HAND_ANGLES

SCAN_TOP_XYZ = 0.3, 0.1, 0.6
SCAN_BOTTOM_XYZ = 0.3, 0.1, 0.1

def parseData( data, robot=None, verbose=False ):
    if len(data) < 5:
        return None
    totalLen, robotState = struct.unpack(">IB", data[:5] )
    assert robotState == 16, robotState
#    print totalLen
    if len(data) < totalLen:
        return None
    ret = data[totalLen:]
    data = data[5:totalLen] # length includes header
    while len(data) > 0:
        subLen, packageType = struct.unpack(">IB", data[:5] )
        if len(data) < subLen:
            return None
#        print packageType, subLen
        if packageType == 0:
            # Robot Mode Data
            assert subLen == 38, subLen
            timestamp, connected, enabled, powerOn, emergencyStopped, protectiveStopped, programRunning, programPaused, \
                robotMode, controlMode, targetSpeedFraction, speedScaling \
                = struct.unpack( ">QBBBBBBBBBdd", data[5:subLen] )
            if robot:
                robot.timestamp = timestamp
            if verbose:
                print timestamp, targetSpeedFraction, speedScaling
        elif packageType == 1:
            # Joint Data
            assert subLen == 251, subLen
            sumSpeed = 0
            for i in xrange(6):
                position, target, speed, current, voltage, temperature, obsolete, mode = \
                        struct.unpack(">dddffffB", data[5+i*41:5+(i+1)*41])
#                print i,speed
                sumSpeed += abs(speed)
                # 253 running mode
            if verbose:
                print "sumSpeed", sumSpeed
            if robot:
                robot.moving = (sumSpeed > 0.000111)
        elif packageType == 2:
            # Tool Data
            assert subLen == 37, subLen
            if verbose:
                print "Tool", struct.unpack(">bbddfBffB", data[5:subLen] )
        elif packageType == 3:
            # Masterboard Data
            assert subLen == 72, subLen
            if verbose:
                print "Masterboard", [hex(x) for x in struct.unpack(">II", data[5:5+8] )]
            if robot:
                robot.inputs, robot.outputs = struct.unpack(">II", data[5:5+8])
        elif packageType == 4:
            # Cartesian Info
            assert subLen == 53, subLen
            x,y,z, rx,ry,rz = struct.unpack( ">dddddd", data[5:subLen] )
            if robot:
                robot.pose = (x,y,z, rx,ry,rz)
            if verbose:
                print "%.3f, %.3f, %.3f,    %.3f, %.3f, %.3f" % (x,y,z, rx,ry,rz)
        data = data[subLen:]
    if verbose:
        print "------------"
    return ret



class UniversalRobotUR5:
    def __init__( self, replayLog=None ):
        self.acc = 0.13962634015954636
        self.speed = 0.30471975511965976
        self.moving = None # unknown
        self.timestamp = None
        self.pose = None
        self.inputs = None
        self.outputs = None
        if replayLog is None:
            self.replayLog = False
            self.s = socket.socket()
            self.s.connect((UR5_HOST, UR5_PORT))
        else:
            self.replayLog = True

    def term( self ):
        if not self.replayLog:
            self.s.close()
            self.s = None

    def receiveData( self ):
        data = self.s.recv(4096)
        # if len(data) > 0:

            # parseData( data, self )

    def sendCmd( self, cmd ):
        self.s.send( cmd )
        self.receiveData()


    def movej( self, sixAngles ):
        assert len(sixAngles) == 6, sixAngles
        self.sendCmd("movej("+str(list(sixAngles)) + ", a=%f, v=%f)" % (self.acc, self.speed) + "\n")
        # or use t=<time> and r=<radius>
#        time.sleep(2.0) # keep it as part of movej? it should rather check positions and wait until it is complete

    def testIO( self ):
#        self.s.send("set_digital_out(8,True)" + "\n") # tool 0
        self.s.send("set_digital_out(8,False)" + "\n") # tool 0
        data = self.s.recv(1024)
        print "Received", repr(data)


    def scan( self ):
        self.sendCmd("movel( p[%f,%f,%f, " % SCAN_TOP_XYZ + HAND_ANGLES_STR + "], a=0.1, v=0.1 )\n")
        for i in xrange(2):
            self.receiveData()
        while self.moving:
            self.receiveData()

        self.sendCmd("movel( p[%f,%f,%f, " % SCAN_BOTTOM_XYZ + HAND_ANGLES_STR + "], a=0.1, v=0.025 )\n")
        for i in xrange(2):
            self.receiveData()
        while self.moving:
            self.receiveData()


    def goto( self, xyz ):
        self.sendCmd( ("movel( p[%f, %f, %f," % xyz) + HAND_ANGLES_STR + "], a=0.1, v=0.1 )\n" )
        for i in xrange(2):
            self.receiveData()
        for i in xrange(200):
            self.receiveData()
            if not self.moving:
                break


    def openGripper( self ):
        for i in xrange(3):
            self.sendCmd("set_digital_out(8,False)" + "\n") # tool 0
            print "OUTPUTS", self.outputs
            if self.outputs == 0:
                break


    def closeGripper( self ):
        for i in xrange(3):
            self.sendCmd("set_digital_out(8,True)" + "\n") # tool 0
            print "OUTPUTS", self.outputs
            if self.outputs != 0:
                break


def testUR5( args ):
    replayLog = None
    robot = UniversalRobotUR5( replayLog )
    robot.testIO()
    # robot.movej( [-0.5405182705025187, -2.350330184112267, -1.316631037266588, -2.2775736604458237, 3.3528323423665642, -1.2291967454894914] )
    # robot.movej( [math.radians(x) for x in [0, -90, 0, -90, 0, 0]] ) # straight position
   # robot.s.send("movej( p[0.2, 0.2, 0.6, 0, -2.2218, 2.2228], a=0.1, v=0.1 )\n") # p - cartesian coordinates :)

# display coordinates
#    robot.s.send('popup("hi")\n')
#    robot.s.send("def myProg():\nm = get_inverse_kin( get_target_tcp_pose())\npopup( m )\nend\nmyProg()\n")
#    robot.s.send("def myProg():\nm = get_target_tcp_pose()\npopup( m )\nend\nmyProg()\n")

#    robot.sendCmd("movej( p[0.143, -0.187, 0.187, 0.16, -2.37, 2.37], a=0.1, v=0.1 )\n")

#    robot.sendCmd("movej( p[-0.00782237301675, -0.228968253334, 0.511269468576, 0.16, -2.37, 2.37], a=0.1, v=0.1 )\n")
#    for i in xrange(10):
#        robot.receiveData()
#    robot.sendCmd("movej( p[0.0786829156856, -0.173025495873, 0.671427332353, 0.16, -2.37, 2.37], a=0.1, v=0.1 )\n")
#    for i in xrange(10):
#        robot.receiveData()

    # robot.openGripper()
    # robot.scan()
    robot.goto( (0.3693, 0.291, 0.279) ) # pick apple
    # robot.closeGripper()
    # robot.goto( (0.3693, 0.291, 0.1) ) # drop apple
    # robot.openGripper()
    # robot.term()
    print robot.pose


if __name__ == "__main__":
    testUR5( sys.argv )
