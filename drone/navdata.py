#!/usr/bin/python
"""
  Parsing of incomming messages from Parrot Bebop
  usage:
       ./navdata.py <logged file>
"""
import sys
import struct

ARNETWORKAL_FRAME_TYPE_ACK = 0x1
ARNETWORKAL_FRAME_TYPE_DATA = 0x2
ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY = 0x3
ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK = 0x4

ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING = 0
ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PONG = 1

POSITION_TIME_DELTA = 0.2 # estimated to 5Hz


def printHex( data, robot=None, debug=True):
    if robot is None:
        print(" ".join(["%02X" % ord(x) for x in data]))
    else:
        if debug:
            robot.logger.debug(" ".join(["%02X" % ord(x) for x in data]))
        else:
            robot.logger.info(" ".join(["%02X" % ord(x) for x in data]))

def parseFrameType( data ):
    if len(data) < 7:
        return None
    frameType, frameId, frameSeq, frameSize = struct.unpack("<BBBI", data[:7])
    assert len(data) == frameSize, (len(data), frameSize)
    return frameType

def cutPacket( data ):
    if len(data) < 7:
        return None, data
    frameType, frameId, frameSeq, frameSize = struct.unpack("<BBBI", data[:7])
    return data[:frameSize], data[frameSize:]


def parseData( data, robot ):
    # m:\git\ARDroneSDK3\libARNetworkAL\Includes\libARNetworkAL\ARNETWORKAL_Frame.h
    #   uint8_t type; /**< frame type eARNETWORK_FRAME_TYPE */
    #   uint8_t id; /**< identifier of the buffer sending the frame */
    #   uint8_t seq; /**< sequence number of the frame */
    #   uint32_t size; /**< size of the frame */
    #   uint8_t *dataPtr; /**< pointer on the data of the frame */
    #
    assert len(data) >= 7, len(data)
    frameType, frameId, frameSeq, frameSize = struct.unpack("<BBBI", data[:7])
    assert frameType in [0x1, 0x2, 0x3, 0x4], frameType # 0x2 = ARNETWORKAL_FRAME_TYPE_DATA,
                                              # 0x4 = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK
    if frameType == ARNETWORKAL_FRAME_TYPE_ACK:
        assert frameSize == 8, frameSize
        assert frameId == 0x8B, hex(frameId)
        # if verbose:
        #     print("ACKACK", ord(data[frameSize-1]))
        robot.logger.debug("ACKACK: %d", ord(data[frameSize-1]))
        data = data[frameSize:]
        return data

    if frameType == ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY: # 0x3
        assert frameSize >= 12, frameSize
        assert frameId == 0x7D, hex(frameId)
        frameNumber, frameFlags, fragmentNumber, fragmentsPerFrame = struct.unpack("<HBBB", data[7:12])
        #print("Video", frameNumber, frameFlags, fragmentNumber,fragmentsPerFrame)
        #printHex( data[:20] )
        data = data[frameSize:]
        return data

    assert frameId in [0x7F, 0x0, 0x7E], frameId

    if frameId == 0x7F:
        # Navdata, sent constantly
        commandProject, commandClass, commandId = struct.unpack("BBH",  data[7:7+4])
        if commandProject == 0:
            if (commandClass, commandId) == (5,7):
                # ARCOMMANDS_ID_PROJECT_COMMON = 0,
                # ARCOMMANDS_ID_COMMON_CLASS_COMMONSTATE = 5,
                # ARCOMMANDS_ID_COMMON_COMMONSTATE_CMD_WIFISIGNALCHANGED = 7,
                rssi = struct.unpack("h", data[7:7+2])[0] # RSSI of the signal between controller and the product (in dbm)
                if robot.navdataVerbose:
                    robot.logger.debug("Wifi: %s", rssi)
            else:
                printHex( data[:frameSize], robot=robot )
        elif commandProject == 1:
            if (commandClass, commandId) == (4,4):
                lat, lon, alt = struct.unpack("ddd", data[11:11+3*8])
                robot.positionGPS = (lat, lon, alt)
                if robot.navdataVerbose:
                    robot.logger.debug("Position: (%f, %f, %f)", lat, lon, alt)
            elif (commandClass, commandId) == (4,5):
                speedX, speedY, speedZ = struct.unpack("fff", data[11:11+3*4])
                robot.speed = (speedX, speedY, speedZ)
                robot.position = robot.position[0]+POSITION_TIME_DELTA*speedX, robot.position[1]+POSITION_TIME_DELTA*speedY, robot.position[2]+POSITION_TIME_DELTA*speedZ
                if robot.navdataVerbose:
                    robot.logger.debug("Speed: (%f, %f, %f)", speedX, speedY, speedZ)
            elif (commandClass, commandId) == (4,6):
                roll, pitch, yaw = struct.unpack("fff", data[11:11+3*4])
                if robot.navdataVerbose:
                    robot.logger.debug("Angle: (%f, %f, %f)", roll, pitch, yaw)
            elif (commandClass, commandId) == (4,8):
                robot.altitude = struct.unpack("d", data[11:11+8])[0]
                if robot.navdataVerbose:
                    robot.logger.debug("Altitude: %f", robot.altitude)
            elif (commandClass, commandId) == (25,0):
                tilt,pan = struct.unpack("BB", data[11:11+2])
                if robot.navdataVerbose:
                    robot.logger.debug("CameraState Tilt/Pan: (%f, %f)", tilt, pan)
            else:
                if robot.navdataVerbose:
                    robot.logger.debug("UNKNOWN")
                    printHex( data[:frameSize], robot=robot )
                    assert False
        else:
            print("UNKNOWN Project", commandProject)
    elif frameId == 0x7E:
        # Events, sent when triggered
        commandProject, commandClass, commandId = struct.unpack("BBH",  data[7:7+4])
        if (commandProject, commandClass) == (0,3):
            # ARCOMMANDS_ID_COMMON_CLASS_SETTINGSSTATE = 3,
            if commandId == 0:
                # ARCOMMANDS_ID_COMMON_SETTINGSSTATE_CMD_ALLSETTINGSCHANGED = 0
                robot.logger.debug("AllSettings - done")
            elif commandId == 1:
                # ARCOMMANDS_ID_COMMON_SETTINGSSTATE_CMD_RESETCHANGED = 1
                robot.logger.debug("ResetChanged")
            elif commandId == 2:
                # ARCOMMANDS_ID_COMMON_SETTINGSSTATE_CMD_PRODUCTNAMECHANGED = 2
                robot.logger.debug("ProductName: %s", data[11:frameSize-1])
            elif commandId == 3:
                # ARCOMMANDS_ID_COMMON_SETTINGSSTATE_CMD_PRODUCTVERSIONCHANGED = 3
                robot.logger.debug("ProductVersion: %s", data[11:frameSize-1])
            elif commandId == 4:
                # ARCOMMANDS_ID_COMMON_SETTINGSSTATE_CMD_PRODUCTSERIALHIGHCHANGED = 4
                robot.logger.debug("ProductSerialHigh: %s", data[11:frameSize-1])
            elif commandId == 5:
                # ARCOMMANDS_ID_COMMON_SETTINGSSTATE_CMD_PRODUCTSERIALLOWCHANGED = 5
                robot.logger.debug("ProductSerialLow: %s", data[11:frameSize-1])
            elif commandId == 6:
                # ARCOMMANDS_ID_COMMON_SETTINGSSTATE_CMD_COUNTRYCHANGED = 6
                robot.logger.debug("Country: %s", data[11:frameSize-1])
            elif commandId == 7:
                # ARCOMMANDS_ID_COMMON_SETTINGSSTATE_CMD_AUTOCOUNTRYCHANGED = 7
                robot.logger.debug("AutoCountry: %s", struct.unpack("B", data[11:12])[0])
            else:
                robot.logger.debug("Unknown(0,3): %d", commandId)
                printHex( data[:frameSize] , robot=robot)
        elif (commandProject, commandClass, commandId) == (0,5,1):
            battery = struct.unpack("B", data[11:12])[0]
            robot.battery = battery
            robot.logger.debug("Battery: %d", battery)
        elif (commandProject, commandClass, commandId) == (0,5,4):
            robot.logger.debug("Date: %s", data[11:frameSize-1])
        elif (commandProject, commandClass, commandId) == (0,5,5):
            robot.logger.debug("Time: %s", data[11:frameSize-1])
        elif (commandProject, commandClass, commandId) == (0,10,0):
            # ARCOMMANDS_ID_COMMON_CLASS_WIFISETTINGSSTATE = 10,
            # ARCOMMANDS_ID_COMMON_WIFISETTINGSSTATE_CMD_OUTDOORSETTINGSCHANGED
            robot.logger.debug("WiFi Outdoor: %d", struct.unpack("B", data[11:12])[0])
        elif (commandProject, commandClass) == (0,14):
            # ARCOMMANDS_ID_COMMON_CLASS_CALIBRATIONSTATE = 14,
            if commandId == 0:
                # ARCOMMANDS_ID_COMMON_CALIBRATIONSTATE_CMD_MAGNETOCALIBRATIONSTATECHANGED = 0,
                x,y,z,failed = struct.unpack("BBBB", data[11:11+4])
                robot.logger.debug("Magnetometer calibration: (%d, %d, %d) %d", x, y, z, failed)
            elif commandId == 1:
                # ARCOMMANDS_ID_COMMON_CALIBRATIONSTATE_CMD_MAGNETOCALIBRATIONREQUIREDSTATE
                required = struct.unpack("B", data[11:11+1])[0]
                robot.logger.debug("Magnetometer calibration required: %d", required)
            elif commandId == 3:
                # ARCOMMANDS_ID_COMMON_CALIBRATIONSTATE_CMD_MAGNETOCALIBRATIONSTARTEDCHANGED
                started = struct.unpack("B", data[11:11+1])[0]
                robot.logger.debug("Magnetometer calibration started: %d", started)
            else:
                robot.logger.debug("Calibration: %d", commandId)
                printHex( data[:frameSize], robot=robot )

        elif (commandProject, commandClass) == (1,4):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_PILOTINGSTATE = 4,
            if commandId == 0:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSTATE_CMD_FLATTRIMCHANGED = 0
                robot.logger.info("FlatTrim changed")
                robot.flatTrimCompleted = True
            elif commandId == 1:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSTATE_CMD_FLYINGSTATECHANGED = 1
                state = struct.unpack("I", data[11:11+4])[0]
                states = ["landed", "takingoff", "hovering", "flying", "landing", "emergency"]
                robot.flyingState = state
                robot.logger.info("Flying State: %d, %s", state, states[state])
            elif commandId == 2:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSTATE_CMD_ALERTSTATECHANGED
                state = struct.unpack("I", data[11:11+4])[0]
                states = ["none/No alert", "user/User emergency alert", "cut_out/Cut out alert", "critical_battery", "low_battery", "too_much_angle"]
                robot.logger.info("ALERT State: %d, %s", state, states[state])
            elif commandId == 3:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSTATE_CMD_NAVIGATEHOMESTATECHANGED
                state, reason = struct.unpack("II", data[11:11+2*4])
                states = ["available", "inProgress", "unavailable", "pending", "low_battery", "too_much_angle"]
                reasons = ["userRequest", "connectionLost", "lowBattery", "finished", "stopped", "disabled", "enabled"]
                robot.logger.info("NavigateHomeStateChanged: %d, %s, %s", state, states[state], reasons[reason])
                robot.navigateHomeState = state
            else:
                robot.logger.info("Unknoqn Piloting State: %d", commandId)
                printHex( data[:frameSize], robot=robot, debug=False )

        elif (commandProject, commandClass) == (1,6):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_PILOTINGSETTINGSSTATE = 6,
            if commandId == 0:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSETTINGSSTATE_CMD_MAXALTITUDECHANGED = 0,
                robot.logger.debug("MaxAltitude: %s", struct.unpack("fff", data[11:11+3*4]))
            elif commandId == 1:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSETTINGSSTATE_CMD_MAXTILTCHANGED = 1,
                robot.logger.debug("MaxTilt: %s", struct.unpack("fff", data[11:11+3*4]))
            elif commandId == 2:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSETTINGSSTATE_CMD_ABSOLUTCONTROLCHANGED,
                robot.logger.debug("AbsoluteControl: %d", struct.unpack("B", data[11:12])[0])
            elif commandId == 3:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSETTINGSSTATE_CMD_MAXDISTANCECHANGED = 3
                robot.logger.debug("MaxDistance: %s", struct.unpack("fff", data[11:11+3*4]))
            elif commandId == 4:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSETTINGSSTATE_CMD_NOFLYOVERMAXDISTANCECHANGED = 4
                state = struct.unpack("B", data[11:12])[0]
                states = ["unlimited", "limited"]
                robot.logger.debug("MaxDistanceLimitationBehavior: %s", states[state])
            elif commandId == 10:
                # ARCOMMANDS_ID_ARDRONE3_PILOTINGSETTINGSSTATE_CMD_BANKEDTURNCHANGED = 10
                state = struct.unpack("B", data[11:12])[0]
                states = ["disabled", "enabled"]
                robot.logger.debug("BankedTurn: %s", states[state])
            else:
                robot.logger.info("Unknown Piloting Settings State: %d", commandId,)
                printHex( data[:frameSize], robot=robot, debug=False )

        elif (commandProject, commandClass, commandId) == (1,8,0):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_MEDIARECORDSTATE = 8,
            # ARCOMMANDS_ID_ARDRONE3_MEDIARECORDSTATE_CMD_PICTURESTATECHANGED = 0,
            state, massStorageId = struct.unpack("BB", data[11:11+2])
            robot.logger.debug("Picture State Changed: %d, %d", state, massStorageId)
        elif (commandProject, commandClass, commandId) == (1,8,1):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_MEDIARECORDSTATE = 8,
            # ARCOMMANDS_ID_ARDRONE3_MEDIARECORDSTATE_CMD_VIDEOSTATECHANGED = 1
            state, massStorageId = struct.unpack("IB", data[11:11+4+1])
            states = ["stopped", "started", "failed", "autostopped"]
            robot.logger.debug("Video State Changed: %s, %d", states[state], massStorageId)

        elif (commandProject, commandClass) == (1,12):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_SPEEDSETTINGSSTATE
            #  ARCOMMANDS_ID_ARDRONE3_SPEEDSETTINGSSTATE_CMD_HULLPROTECTIONCHANGED = 2,
            #  ARCOMMANDS_ID_ARDRONE3_SPEEDSETTINGSSTATE_CMD_OUTDOORCHANGED = 3,
            if commandId == 0:
            #  ARCOMMANDS_ID_ARDRONE3_SPEEDSETTINGSSTATE_CMD_MAXVERTICALSPEEDCHANGED = 0,
                robot.logger.debug("MaxVerticalSpeed: %s",  struct.unpack("fff", data[11:11+3*4]))
            elif commandId == 1:
            #  ARCOMMANDS_ID_ARDRONE3_SPEEDSETTINGSSTATE_CMD_MAXROTATIONSPEEDCHANGED,
                robot.logger.debug("MaxRotationSpeed: %s", struct.unpack("fff", data[11:11+3*4]))

        elif (commandProject, commandClass) == (1,16):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_SETTINGSSTATE = 16,
            if commandId == 1:
                # ARCOMMANDS_ID_ARDRONE3_SETTINGSSTATE_CMD_PRODUCTMOTORVERSIONLISTCHANGED = 0
                # Deprecated
                pass
            elif commandId == 2:
                # ARCOMMANDS_ID_ARDRONE3_SETTINGSSTATE_CMD_PRODUCTGPSVERSIONCHANGED = 1
                robot.logger.debug("ProductGPSVersionChanged: %s", data[11:frameSize-1])
            elif commandId == 3:
                # ARCOMMANDS_ID_ARDRONE3_SETTINGSSTATE_CMD_MOTORERRORSTATECHANGED = 2
                # id, error = struct.unpack("Bi", data[11:11+1+4])
                # errors = ["noError", "errorEEPRom", "errorMotorStalled", "errorPropellerSecurity",
                #           "errorCommLost", "errorRCEmergencyStop", "errorRealTime",
                #           "errorMotorSetting", "errorTemperature", "errorBatteryVoltage",
                #           "errorLipoCells", "errorMOSFET", "errorBootloader", "errorAssert"]
                # print(id, error)
                # robot.logger.debug("MotorErrorStateChanged: %d, %s", id, errors[error])
                # Broken, needs fixing
                pass
            elif commandId == 4:
                # ARCOMMANDS_ID_ARDRONE3_SETTINGSSTATE_CMD_MOTORFLIGHTSSTATUSCHANGED = 4,
                nbFlights, lastFlightDuration, totalFlightDuration = struct.unpack("HHI", data[11:11+8])
                robot.logger.debug("Motor flights status: (%d, %d, %d)", nbFlights, lastFlightDuration, totalFlightDuration)
            elif commandId == 5:
                # ARCOMMANDS_ID_ARDRONE3_SETTINGSSTATE_CMD_MOTORERRORLASTERRORCHANGED = 5
                lastError = struct.unpack("I", data[11:11+4])[0]
                robot.logger.debug("Motor last error: %d", lastError)
            elif commandId == 6:
                # ARCOMMANDS_ID_ARDRONE3_SETTINGSSTATE_CMD_P7ID = 6
                # Deprecated
                pass
            else:
                robot.logger.debug("Settings state %d", commandId)
                printHex( data[:frameSize], robot=robot )

        elif (commandProject, commandClass, commandId) == (1,20,5):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_PICTURESETTINGSSTATE = 20,
            robot.logger.debug("VIDEOAUTORECORDCHANGED: %s", struct.unpack("BB", data[11:11+2]))

        elif (commandProject, commandClass, commandId) == (1,22,0):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_MEDIASTREAMINGSTATE = 22,
            # ARCOMMANDS_ID_ARDRONE3_MEDIASTREAMINGSTATE_CMD_VIDEOENABLECHANGED = 0,
            state = struct.unpack("I", data[11:11+4])[0]
            states = ["enabled", "disabled", "error"]
            robot.logger.debug("Video Enabled State: %d, %s", state, states[state])

        elif (commandProject, commandClass, commandId) == (1,24,0):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_GPSSETTINGSSTATE = 24,
            # ARCOMMANDS_ID_ARDRONE3_GPSSETTINGSSTATE_CMD_HOMECHANGED = 0,
            robot.logger.debug("Home changed: %s", struct.unpack("dd", data[11:11+16]))

        elif (commandProject, commandClass, commandId) == (1,24,2):
            # ARCOMMANDS_ID_ARDRONE3_CLASS_GPSSETTINGSSTATE = 24,
            # ARCOMMANDS_ID_ARDRONE3_GPSSETTINGSSTATE_CMD_GPSFIXSTATECHANGED = 2,
            robot.logger.debug("GPSFixStateChanged - fixed: %d", struct.unpack("B", data[11:11+1])[0])

        elif (commandProject, commandClass, commandId) == (129,3,0):
            robot.logger.debug("GPSDebugState, numSat = %d", struct.unpack("B", data[11:11+1])[0])

        elif commandProject == 129:
            robot.logger.debug("DEBUG")
            printHex( data[:frameSize], robot=robot)
        else:
            if robot.navdataVerbose:
                robot.logger.debug("Unknown ACK: %d, %d, %d", commandProject, commandClass, commandId)
                printHex( data[:frameSize], robot=robot )

    elif frameId == 0x0:
        # ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING, sent constantly
        assert frameSize == 15, len(data)
        seconds, nanoseconds = struct.unpack("<II", data[7:15])
        assert nanoseconds < 1000000000, nanoseconds
        timestamp = seconds + nanoseconds/1000000000.
        robot.time = timestamp
        if robot.navdataVerbose:
            robot.logger.debug("Time: %d" , timestamp)
    data = data[frameSize:]
    return data


def ackRequired( data ):
    return parseFrameType( data ) == ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK


def createAckPacket( data ):
    assert len(data) >= 7, len(data)
    frameType, frameId, frameSeq, frameSize = struct.unpack("<BBBI", data[:7])
    assert frameType == 0x4, frameType
    assert len(data) == frameSize, (len(data), frameSize)

    # get the acknowledge sequence number from the data
#    payload = data[7:8] # strange
    payload = struct.pack("B", frameSeq)
#    payload = struct.pack("B", 1)
#    print("ACK", repr(payload))

    frameType = ARNETWORKAL_FRAME_TYPE_ACK
    frameId = 0xFE # 0x7E + 0x80
    buf = struct.pack("<BBBI", frameType, frameId, 0, len(payload)+7)
    return buf + payload


def pongRequired( data ):
    if len(data) < 7:
        return False
    frameType, frameId, frameSeq, frameSize = struct.unpack("<BBBI", data[:7])
    return frameId == ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING

def createPongPacket( data ):
    assert len(data) >= 7, len(data)
    frameType, frameId, frameSeq, frameSize = struct.unpack("<BBBI", data[:7])
    assert frameType == 0x2, frameType
    assert frameId == ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PING, frameId
    assert len(data) == frameSize, (len(data), frameSize)

    payload = data[7:]
    frameType = 2
    frameId = ARNETWORK_MANAGER_INTERNAL_BUFFER_ID_PONG
    buf = struct.pack("<BBBI", frameType, frameId, 0, len(payload)+7)
    return buf + payload


def videoAckRequired( data ):
    if len(data) < 7:
        return False
    frameType, frameId, frameSeq, frameSize = struct.unpack("<BBBI", data[:7])
    return frameType == ARNETWORKAL_FRAME_TYPE_DATA_LOW_LATENCY and frameId == 0x7D

g_currentVideoFrameNumber = None
g_lowPacketsAck = 0
g_highPacketsAck = 0

def createVideoAckPacket( data ):
    global g_currentVideoFrameNumber, g_lowPacketsAck, g_highPacketsAck

    assert len(data) >= 12, len(data)
    frameNumber, frameFlags, fragmentNumber, fragmentsPerFrame = struct.unpack("<HBBB", data[7:12])

    if frameNumber != g_currentVideoFrameNumber:
        g_lowPacketsAck = 0
        g_highPacketsAck = 0
        g_currentVideoFrameNumber = frameNumber

    if fragmentNumber < 64:
        g_lowPacketsAck |= (1<<fragmentNumber)
    else:
        g_highPacketsAck |= (1<<(fragmentNumber-64))


    payload = struct.pack("<HQQ", frameNumber, g_highPacketsAck, g_lowPacketsAck )
    frameType = 2
    frameId = 13
    buf = struct.pack("<BBBI", frameType, frameId, 0, len(payload)+7)
    return buf + payload


# vim: expandtab sw=4 ts=4

