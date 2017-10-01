#!/usr/bin/python
"""
  Basic class for communication to Parrot Bebop
  usage:
       ./bebop.py <task> [<metalog> [<F>]]
"""
import sys
import socket
import datetime
import struct
import time
import numpy
import subprocess as sp
import logging

from navdata import *
from commands import *

# this will be in new separate repository as common library fo robotika Python-powered robots
from apyros.metalog import MetaLog, disableAsserts
from apyros.manual import myKbhit, ManualControlException


# hits from https://github.com/ARDroneSDK3/ARSDKBuildUtils/issues/5


HOST = "192.168.42.1"
DISCOVERY_PORT = 44444
NAVDATA_PORT = 43210 # d2c_port
COMMAND_PORT = 54321 # c2d_port

class JpegReader(Thread):
    def __init__(self, drone, fps):
        Thread.__init__(self)
        self.drone = drone
        self.fps = fps
        self.interval = 0
        if self.fps > 0:
            self.interval = 1000000 // self.fps
        self.command = ["ffmpeg", '-i', 'bebop.sdp', '-f', 'image2pipe', '-pix_fmt', 'bgr24', '-q:v', '1', '-vcodec', 'rawvideo', '-'] # to stop FFMPEG output add these to the array after bebop.sdp '-loglevel', 'quiet',
        self.ffmpeg = sp.Popen(self.command, stdout=sp.PIPE, bufsize=10 ** 8)
        self.done = False

    def run(self):
        ms = datetime.datetime.now()
        sum = 0
        while not self.done:
            raw_image = self.ffmpeg.stdout.read(856 * 480 * 3)
            if len(raw_image) != 856 * 480 * 3:
                break
            image = numpy.fromstring(raw_image, dtype='uint8')
            image = image.reshape((480, 856, 3))

            diff = datetime.datetime.now() - ms
            sum = sum + diff.microseconds

            if self.drone.videoCallBack and sum > self.interval:
                self.drone.videoCallBack(image)

            if sum > self.interval:
                sum = 0
            ms = datetime.datetime.now()

    def stop(self):
        done = True
        print("Killing ffmpeg")
        self.ffmpeg.kill()

class Bebop:

    def __init__(self, metalog=None, fps=30, loggingLevel=logging.INFO, navdataVerbose=False):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(loggingLevel)
        if metalog is None:
            self._discovery()
            metalog = MetaLog()
        self.navdata = metalog.createLoggedSocket( "navdata", headerFormat="<BBBI" )
        self.navdata.bind( ('',NAVDATA_PORT) )
        if metalog.replay:
            self.commandSender = CommandSenderReplay(metalog.createLoggedSocket( "cmd", headerFormat="<BBBI" ),
                    hostPortPair=(HOST, COMMAND_PORT), checkAsserts=metalog.areAssertsEnabled())
        else:
            self.commandSender = CommandSender(metalog.createLoggedSocket( "cmd", headerFormat="<BBBI" ),
                    hostPortPair=(HOST, COMMAND_PORT))
        self.console = metalog.createLoggedInput( "console", myKbhit ).get
        self.metalog = metalog
        self.fps = fps
        self.buf = ""
        self.videoStartCallback = None
        self.videoEndCallBack = None
        self.videoCallBack = None
        self.battery = None
        self.flyingState = None
        self.flatTrimCompleted = False
        self.manualControl = False
        self.time = None
        self.altitude = None
        self.position = (0,0,0)
        self.speed = (0,0,0)
        self.positionGPS = None
        self.cameraTilt, self.cameraPan = 0,0
        self.lastImageResult = None
        self.navigateHomeState = None
        self.navdataVerbose = navdataVerbose
        self.config()
        self.commandSender.start()

        # Plate finding variables
        self.findPlate = False


        # Sphero tracking variables
        self.findSphero = False
        self.sinceLastSphero = 0
        self.foundCircle = False
        self.lastFrame = None
        self.thisFrame = None
        self.moveScaler = 0
        self.objectCenterX = 0
        self.objectCenterY = 0
        self.minEdgeVal = 0
        self.maxEdgeVal = 0
        self.minCircleRadius = 0
        self.maxCircleRadius = 0
        self.frameWidth = 0
        self.frameHeight = 0


    def _discovery( self ):
        self.logger.info("Discovering drone...")
        "start communication with the robot"
        filename = "tmp.bin" # TODO combination outDir + date/time
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
        s.connect( (HOST, DISCOVERY_PORT) )
        s.send( '{"controller_type":"computer", "controller_name":"katarina", "controller_type": "Unix", "d2c_port":"43210", "arstream2_client_stream_port": 55004, "arstream2_client_control_port": 55005 }' )
        f = open( filename, "wb" )
        while True:
            data = s.recv(10240)
            if len(data) > 0:
                f.write(data)
                f.flush()
                break
        f.close()
        s.close()

    def _update( self, cmd ):
        "internal send command and return navdata"

        if not self.manualControl:
            self.manualControl = self.console()
            if self.manualControl:
                # raise exception only once
                raise ManualControlException()

        # send even None, to sync in/out queues
        self.commandSender.send( cmd )

        while len(self.buf) == 0:
            data = self.navdata.recv(40960)
            self.buf += data
        data, self.buf = cutPacket( self.buf )
        return data

    def _parseData( self, data ):
        try:
            parseData( data, robot=self )
        except AssertionError, e:
            self.logger.info("AssertionError: %s", e)


    def update( self, cmd=None, ackRequest=False ):
        "send command and return navdata"
        if cmd is None:
            data = self._update( None )
        else:
            data = self._update( packData(cmd, ackRequest=ackRequest) )
        while True:
            if ackRequired(data):
                self._parseData( data )
                data = self._update( createAckPacket(data) )
            elif pongRequired(data):
                self._parseData( data ) # update self.time
                data = self._update( createPongPacket(data) )
            else:
                break
        self._parseData( data )
        return data

    def config( self ):
        # initial cfg
        dt = self.metalog.now()
        if dt: # for compatibility with older log files
            self.update( cmd=setDateCmd( date=dt.date() ) )
            self.update( cmd=setTimeCmd( time=dt.time() ) )
        for cmd in setSpeedSettingsCmdList( maxVerticalSpeed=1.0, maxRotationSpeed=90.0,
                hullProtection=True, outdoor=True ):
            self.update( cmd=cmd )
        for cmd in setPilotingSettingsCmdList( maxAltitude=10.0, maxTilt=15.0, maxDistance=2000.0):
            self.update( cmd=cmd )
        self.update( cmd=requestAllStatesCmd() )
        self.update( cmd=requestAllSettingsCmd() )
        self.moveCamera( tilt=self.cameraTilt, pan=self.cameraPan )
        self.update( videoAutorecordingCmd( enabled=False ) )


    def takeoff( self ):
        self.update( videoRecordingCmd( on=True ) )
        for i in xrange(10):
            self.update( cmd=None )
        self.logger.info("Taking off...")
        self.update( cmd=takeoffCmd() )
        prevState = None
        for i in xrange(100):
            self.update( cmd=None )
            if self.flyingState != 1 and prevState == 1:
                break
            prevState = self.flyingState
        self.logger.info("FLYING")

    def land( self ):
        self.logger.info("Landing...")
        self.update( cmd=landCmd() )
        for i in xrange(100):
            self.update( cmd=None )
            if self.flyingState == 0: # landed
                break
        self.logger.info("LANDED")
        self.update( videoRecordingCmd( on=False ) )
        for i in xrange(30):
            self.update( cmd=None )

    def hover( self ):
        self.update( cmd=movePCMDCmd( active=True, roll=0, pitch=0, yaw=0, gaz=0 ) )

    def emergency( self ):
        self.update( cmd=emergencyCmd() )

    def trim( self ):
        self.logger.info("Trim")
        self.flatTrimCompleted = False
        for i in xrange(10):
            self.update( cmd=None )
        self.update( cmd=trimCmd() )
        for i in xrange(10):
            self.update( cmd=None )
            if self.flatTrimCompleted:
                break

    def takePicture( self ):
        self.update( cmd=takePictureCmd() )

    def video_callbacks(self, start, end, image):
        self.videoStartCallback = start
        self.videoEndCallBack = end
        self.videoCallBack = image

    def videoEnable( self ):
        "enable video stream"
        self.update( cmd=videoStreamingCmd( enable=True ), ackRequest=True )
        if self.videoStartCallback is not None:
            self.videoStartCallback()

        self.reader = JpegReader(self, self.fps)
        self.reader.start()

    def videoDisable( self ):
        "enable video stream"
        self.reader.stop()
        self.update( cmd=videoStreamingCmd( enable=False ), ackRequest=True )
        if self.videoEndCallBack is not None:
            self.videoEndCallBack()

    def moveCamera( self, tilt, pan ):
        "Tilt/Pan camera consign for the drone (in degrees)"
        self.update( cmd=moveCameraCmd( tilt=tilt, pan=pan) )
        self.cameraTilt, self.cameraPan = tilt, pan # maybe move this to parse data, drone should confirm that

    def resetHome( self ):
        self.update( cmd=resetHomeCmd() )


    def wait( self, duration ):
        self.logger.info("Wait: %d", duration)
        assert self.time is not None
        startTime = self.time
        while self.time-startTime < duration:
            self.update()

    def flyToAltitude( self, altitude, timeout=3.0 ):
        self.logger.info("Fly to altitude %f from %f", altitude, self.altitude)
        speed = 20 # 20%
        assert self.time is not None
        assert self.altitude is not None
        startTime = self.time
        if self.altitude > altitude:
            while self.altitude > altitude and self.time-startTime < timeout:
                self.update( movePCMDCmd( True, 0, 0, 0, -speed ) )
        else:
            while self.altitude < altitude and self.time-startTime < timeout:
                self.update( movePCMDCmd( True, 0, 0, 0, speed ) )
        self.update( movePCMDCmd( True, 0, 0, 0, 0 ) )

# vim: expandtab sw=4 ts=4

