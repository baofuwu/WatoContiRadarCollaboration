import socket
import struct
import ctypes
import datetime
import os
from decimal import Decimal
import time, sys
from multiprocessing import Process
from socket import AF_PACKET, SOCK_RAW

import math
#import 'radar_receive_data_h'


OFFLINE    =      0
LIVE       =      1

# #Logging Definitions
LOGGING =         0

# #Packet Print Definitions
PRINT_BOTH =      0 #RDI & SS
PRINT_RDI  =      1 #RDI only
PRINT_SS   =      2 #SS only
PRINT_HDR  =      3 #Only Header
PRINT_ALL  =      4 #RDI, SS, and Header
PRINT_NONE =      5 #Nothing

# #Service ID Definitions
RDI_PACKET_ID =   220
SS_PACKET_ID  =   200

# # Event ID Definitions
FAR0   =  1
FAR1   =  2
NEAR0  =  3
NEAR1  =  4
NEAR2  =  5

# #General Definitions
SENSOR_SERIAL_NUM_LEN  = 26
RDI_ARRAY_LEN       =    38
RDI_ARRAY_LEN_NEAR_2  =  32

# #Size Defs
NUM_FAR     =    12
NUM_NEAR    =    18

# #Pdh0 Probability flag masks
# #Usage: bool x = pdh0 & NEAR_PROB_MASK
BEAM7_PROB_MASK     =  64
BEAM6_PROB_MASK     =  32
CLUSTER_PROB_MASK   =  16
DOPPLER_PROB_MASK   =  8
SIDELOBE_PROB_MASK  =  4
INFERENCE_PROB_MASK =  2
NEAR_PROB_MASK      =  1

# # flag masks for SS Packet fields.
DEFECTIVE_HW    =    1
BAD_VOLTAGE     =    1
BAD_TEMP        =    1
GM_MISSING      =    1
POWER_REDUCED   =    1


##Return Values
SUCCESS         =    0
NO_DETECTIONS    =   1
PUBLISH_FAIL      =  2
CLEAR_FAIL      = 3
BAD_EVENT_ID    =    4
NO_PUBLISHER    =    5
INIT_FAIL       =    6
BAD_PORT        =    7
BAD_SERVICE_ID  =    8
NO_PUBLISHER    =    9
NO_PUB_CLR_FAIL =    10
FALSE_DETECTION_1 =  11
FALSE_DETECTION_2  = 12
FALSE_DETECTION_3  = 13
TOO_MUCH_NOISE     = 14
SS_DEFECTIVE_HW    = 15
SS_BAD_VOLT        = 16
SS_BAD_TEMP        = 17
SS_GM_MISSING      = 18
SS_PWR_REDUCED     = 19
NO_PROCESS         = 20

##Filter Thresholds
SNR_THRESHOLD       =     3 ##dBr
VELOCITY_LOWER_THRESHOLD = 0.00 ## m/s
RCS_THRESHOLD         =  -70 ##(dBm)^2
DISTANCE_MAX_THRESHOLD =  70 ##m
DISTANCE_MIN_THRESHOLD  = 0.25 ##m


f_RangeRes      = 0.004577776
f_VrelRadRes    = 0.004577776
f_AzAng0Res     = 0.0000958767
f_AzAng1Res     = 0.0000958767
f_ElAngRes      = 0.0000958767
f_RCS0Res       = 0.003051851
f_RCS1Res       = 0.003051851
f_Prob0Res      = 0.003937008
f_Prob1Res      = 0.003937008
f_RangeVarRes   = 0.000152593
f_VrelRadVarRes = 0.000152593
f_AzAngVar0Res  = 0.0000152593
f_AzAngVar1Res  = 0.0000152593
f_ElAngVarRes   = 0.0000152593
f_Pdh0Res       = 3 ##placeholder value, this data type is different to decode
f_SNRRes        = 0.1
VambigRes       = 0.0030519
CenterFreqRes   = 0.05



# class radarFarFullScanMessage:
#     def __init__(self):
#         farScanTimeStamp = ""
#         self.far = []

# class radarNearFullScanMessage:
#     def __init__(self):
#         nearScanTimeStamp = ""
#         self.near = []

class NewMsg :
    def __init__(self):
        self.header =0
        self.EventID =0
        self.TimeStamp = 0
        self.MeasurementCounter = 0
        self.Vambig =0
        self.CenterFrequency =0
        self.Detections = []

class Data :
    def __init__(self):
        self.posX = 0
        self.posY = 0
        self.posZ = 0
        self.VrelRad = 0
        self.AzAng = 0
        self.ElAng = 0
        self.RCS = 0
        self.RangeVar = 0
        self.VrelRadVar = 0
        self.AzAngVar = 0
        self.ElAngVar = 0
        self.SNR = 0

class PacketGroup:
    def __init__(self) :
        self.nearPackets = []
        self.farPackets  = []
        self.numFarPackets  = 0
        self.numNearPackets = 0

def loadRDIMessageFromPacket(newMsg, oldMsg):

    newMsg.EventID                    = oldMsg.EventID
    newMsg.TimeStamp                  = oldMsg.TimeStamp
    newMsg.MeasurementCounter         = oldMsg.MeasurementCounter
    newMsg.Vambig                     = oldMsg.Vambig
    newMsg.CenterFrequency            = oldMsg.CenterFrequency
    newMsg.Detections.clear()

    for  i in range(oldMsg.Detections.size()) :

        ## TODO: Figure out an SNR threshold value that actually works here.
        if oldMsg.Detections[i].SNR < SNR_THRESHOLD : ## Too much noise drop detection.
            continue
        elif abs(oldMsg.Detections[i].VrelRad) < VELOCITY_LOWER_THRESHOLD:
            continue
        elif oldMsg.Detections[i].posX > DISTANCE_MAX_THRESHOLD: ## need to do trig
            continue
        elif oldMsg.Detections[i].posX < DISTANCE_MIN_THRESHOLD:
            continue
        
        tmpdata = Data()

        ## move check prob to parser.cpp
        tmpdata.AzAng        = oldMsg.Detections[i].AzAng
        tmpdata.RCS          = oldMsg.Detections[i].RCS
        tmpdata.AzAngVar     = oldMsg.Detections[i].AzAngVar
        tmpdata.VrelRad      = oldMsg.Detections[i].VrelRad
        tmpdata.ElAng        = 0 ##oldMsg.Detections[i].f_ElAng ##Told to ignore by continental
        tmpdata.RangeVar     = oldMsg.Detections[i].RangeVar
        tmpdata.VrelRadVar   = oldMsg.Detections[i].VrelRadVar
        tmpdata.ElAngVar     = oldMsg.Detections[i].ElAngVar
        tmpdata.SNR          = oldMsg.Detections[i].SNR

        ## VEEEERY SIMPLIFIED, probably will be more complex/accurate than this
        tmpdata.posX = oldMsg.Detections[i].posX
        tmpdata.posY = -1 * tmpdata.posX * math.tan(tmpdata.AzAng)  ##Flip y axis
        tmpdata.posZ = tmpdata.posX * math.tan(tmpdata.ElAng)

        newMsg.Detections.push_back(tmpdata)

    ## Return true if there is at least one detection in newMsg after filtering
    return (newMsg.Detections.size() > 0)



def receiveAndPackAndSendRadarData(sock, index):
    nearScanTimeStamp = 0
    farScanTimeStamp  = 0
    frame_id = 0

    radargroup = PacketGroup()
    curGroup = PacketGroup()
    # Init Double Buffer
    PacketsBuffer = [PacketGroup() for i in range(2)]

    # print(f"len of PacketsBuffer:{len(PacketsBuffer)}")

    msg = NewMsg()  
 #   tmpdata = Data()

    curFarIdx = 0
    curNearIdx = 0
    publish = False

       
    while True:
        frame_id += 1        
        data, addr = sock.recvfrom(2000)  # buffer size is 2000 bytes      
        HeaderID = int.from_bytes(data[0:4],byteorder= 'big')
        SOMEIPLength =int.from_bytes(data[4:8],byteorder= 'big')        
        ClientID = int.from_bytes(data[8:10], byteorder='big')
        SessionID = int.from_bytes(data[10:12], byteorder='big')
        ProtocolVersion = int.from_bytes(data[12:13], byteorder='big')
        InterfaceVersion =  int.from_bytes(data[13:14], byteorder='big')
        MessageType = int.from_bytes(data[14:15], byteorder='big')
        ReturnCode = int.from_bytes(data[15:16], byteorder='big')        
        E2EP06_CRC = int.from_bytes(data[16:18], byteorder='big')
        E2EP06_Length = int.from_bytes(data[18:20], byteorder='big')
        E2EP06_Counter = int.from_bytes(data[20:21], byteorder='big')        
        if HeaderID == 13107200:
            if(sendStatus == False):
            	continue           
            startIndex = 21
            
            SensorStatus_partNumber = int.from_bytes(data[startIndex:startIndex+8], byteorder='big', signed=False)
            startIndex += 8
            SensorStatus_assemblyPartNumber = int.from_bytes(data[startIndex:startIndex+8], byteorder='big', signed=False)
            startIndex += 8
            SensorStatus_swPartNumber = int.from_bytes(data[startIndex:startIndex+8], byteorder='big', signed=False)
            startIndex += 8
            SensorStatus_serialNumber = data[startIndex:startIndex+26]
            startIndex += 26
            SensorStatus_blVersion = int.from_bytes(data[startIndex:startIndex+3], byteorder='big', signed=False)
            startIndex += 3
            SensorStatus_blCRC = int.from_bytes(data[startIndex:startIndex+4], byteorder='big', signed=False)
            startIndex += 4
            SensorStatus_swVersion = int.from_bytes(data[startIndex:startIndex+3], byteorder='big', signed=False)
            startIndex += 3
            SensorStatus_swCRC = int.from_bytes(data[startIndex:startIndex+4], byteorder='big', signed=False)
            startIndex += 4
            SensorStatus_utcTimeStamp = int.from_bytes(data[startIndex:startIndex+8], byteorder='big', signed=False)
            startIndex += 8
            SensorStatus_timeStamp = int.from_bytes(data[startIndex:startIndex+4], byteorder='big', signed=False)
            startIndex += 4
            SensorStatus_currentDamping = int.from_bytes(data[startIndex:startIndex+4], byteorder='big', signed=False)
            startIndex += 4
            SensorStatus_opState = int.from_bytes(data[startIndex:startIndex+1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_currentFarCF = int.from_bytes(data[startIndex:startIndex+1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_currentNearCF = int.from_bytes(data[startIndex:startIndex+1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_defective = int.from_bytes(data[startIndex:startIndex+1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_supplViltLimit = int.from_bytes(data[startIndex:startIndex+1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_SensorOffTemp = int.from_bytes(data[startIndex:startIndex+1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_gmMissing = int.from_bytes(data[startIndex:startIndex+1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_txOutReduced = int.from_bytes(data[startIndex:startIndex+1], byteorder='big', signed=False)
            startIndex += 1
            SensorStatus_maximumRangeFar = int.from_bytes(data[startIndex:startIndex+2], byteorder='big', signed=False)
            startIndex += 2
            SensorStatus_maximumRangeNear = int.from_bytes(data[startIndex:startIndex+2], byteorder='big', signed=False)

            #Status Message fields are received

        elif HeaderID == 14417921 or HeaderID == 14417922 \
                or HeaderID == 14417923 or HeaderID == 14417924 or HeaderID == 14417925:
            startIndex = 21
            
            messageCounter = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            utcTimeStamp = int.from_bytes(data[startIndex:startIndex + 8], byteorder='big', signed=False)
            startIndex += 8
            timeStamp = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            mesurmentCounter = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            cycleCounter = int.from_bytes(data[startIndex:startIndex + 4], byteorder='big', signed=False)
            startIndex += 4
            nofDetections = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
            startIndex += 2
            vAmbig = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
            startIndex += 2
            centerFrequency = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            detectionsInPacket = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
            startIndex += 1
            # print(f"detectionsInPacket:{detectionsInPacket}")
            radarDetectionList = []
            for i in range(detectionsInPacket):
                f_Range = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_VrelRad = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_AzAng0 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_AzAng1 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_ElAng = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_RCS0 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_RCS1 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=True)
                startIndex += 2
                f_Prob0 = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                f_Prob1 = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                f_RangeVar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_VrelRadVar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_AzAngVar0 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_AzAngVar1 = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_ElAngVar = int.from_bytes(data[startIndex:startIndex + 2], byteorder='big', signed=False)
                startIndex += 2
                f_Pdh0 = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                f_SNR = int.from_bytes(data[startIndex:startIndex + 1], byteorder='big', signed=False)
                startIndex += 1
                #radarDetectionList can be appended by an object created from all the extracted values in the current for-loop iteration
                
                ## Add in the proper resolution
                f_Range      = f_Range      * f_RangeRes
                f_VrelRad    = f_VrelRad    * f_VrelRadRes
                f_AzAng0     = f_AzAng0     * f_AzAng0Res
                f_AzAng1     = f_AzAng1     * f_AzAng1Res
                f_ElAng      = f_ElAng      * f_ElAngRes
                f_RCS0       = f_RCS0       * f_RCS0Res
                f_RCS1       = f_RCS1       * f_RCS1Res
                f_Prob0      = f_Prob0      * f_Prob0Res
                f_Prob1      = f_Prob1      * f_Prob1Res
                f_RangeVar   = f_RangeVar   * f_RangeVarRes
                f_VrelRadVar = f_VrelRadVar * f_VrelRadVarRes
                f_AzAngVar0  = f_AzAngVar0  * f_AzAngVar0Res
                f_AzAngVar1  = f_AzAngVar1  * f_AzAngVar1Res
                f_ElAngVar   = f_ElAngVar   * f_ElAngVarRes
                f_Pdh0       = f_Pdh0
                f_SNR        = f_SNR        * f_SNRRes
                
                # print(f"f_Range:{f_Range}")
            #  AzAng0 or AzAng1

                msg.EventID                    = HeaderID
                msg.TimeStamp                  = utcTimeStamp
                msg.MeasurementCounter         = mesurmentCounter
                msg.Vambig                     = vAmbig
                msg.CenterFrequency            = centerFrequency
                msg.Detections.clear()

                
                #Filter through error flags in packet
                if f_Pdh0 and NEAR_PROB_MASK :
                    continue
                elif f_Pdh0 and INFERENCE_PROB_MASK :
                    continue
                elif f_Pdh0 and SIDELOBE_PROB_MASK : 
                    continue

                # tmpdata = Data()

                # if f_Prob0 >= f_Prob1 :
                #     #printf("RCS0: %2.06f, Velocity: %2.06f, SNR: %u \r\n", f_RCS0, f_VrelRad, listDataArray[i].f_SNR)
                #     tmpdata.AzAng        = f_AzAng0
                #     tmpdata.RCS          = f_RCS0
                #     tmpdata.AzAngVar     = f_AzAngVar0
                # else:
                #     #printf("RCS1: %2.06f, Velocity: %2.06f, SNR: %u \r\n", f_RCS1, presListDataArray[i].f_VrelRad,listDataArray[i].f_SNR)
                #     tmpdata.AzAng        = f_AzAng1
                #     tmpdata.RCS          = f_RCS1
                #     tmpdata.AzAngVar     = f_AzAngVar1

                # print(f"f_SNR:{f_SNR},SNR_THRESHOLD:{SNR_THRESHOLD},abs(f_VrelRad):{abs(f_VrelRad)},VELOCITY_LOWER_THRESHOLD:{VELOCITY_LOWER_THRESHOLD},f_Range:{f_Range},DISTANCE_MAX_THRESHOLD:{DISTANCE_MAX_THRESHOLD},DISTANCE_MIN_THRESHOLD:{DISTANCE_MIN_THRESHOLD}")

                # # TODO: Figure out an SNR threshold value that actually works here.
                # if f_SNR < SNR_THRESHOLD : # Too much noise drop detection.
                #     continue
                # elif abs(f_VrelRad) < VELOCITY_LOWER_THRESHOLD :
                #     continue
                # elif f_Range > DISTANCE_MAX_THRESHOLD : # need to do trig
                #     continue
                # elif f_Range < DISTANCE_MIN_THRESHOLD :
                #     continue
                # print(f"success")
                # # move check prob to parser.cpp
                # tmpdata.VrelRad      = f_VrelRad
                # tmpdata.ElAng        = 0       #f_ElAng #Told to ignore by continental
                # tmpdata.RangeVar     = f_RangeVar
                # tmpdata.VrelRadVar   = f_VrelRadVar
                # tmpdata.ElAngVar     = f_ElAngVar
                # tmpdata.SNR          = f_SNR

                # # VEEEERY SIMPLIFIED, probably will be more complex#accurate than this
                # tmpdata.posX = f_Range
                # tmpdata.posY = -1 * tmpdata.posX * math.tan(tmpdata.AzAng)  #Flip y axis
                # tmpdata.posZ = tmpdata.posX * math.tan(tmpdata.ElAng)

                # print(f"tmpdata:{tmpdata}")

                # msg.Detections.extend(tmpdata)
                # print(f"msg:{msg}")


                if f_Prob0 >= f_Prob1 :
                    #printf("RCS0: %2.06f, Velocity: %2.06f, SNR: %u \r\n", f_RCS0, f_VrelRad, listDataArray[i].f_SNR)
                    AzAng        = f_AzAng0
                    RCS          = f_RCS0
                    AzAngVar     = f_AzAngVar0
                else:
                    #printf("RCS1: %2.06f, Velocity: %2.06f, SNR: %u \r\n", f_RCS1, presListDataArray[i].f_VrelRad,listDataArray[i].f_SNR)
                    AzAng        = f_AzAng1
                    RCS          = f_RCS1
                    AzAngVar     = f_AzAngVar1

            #    print(f"f_SNR:{f_SNR},SNR_THRESHOLD:{SNR_THRESHOLD},abs(f_VrelRad):{abs(f_VrelRad)},VELOCITY_LOWER_THRESHOLD:{VELOCITY_LOWER_THRESHOLD},f_Range:{f_Range},DISTANCE_MAX_THRESHOLD:{DISTANCE_MAX_THRESHOLD},DISTANCE_MIN_THRESHOLD:{DISTANCE_MIN_THRESHOLD}")

                # TODO: Figure out an SNR threshold value that actually works here.
                if f_SNR < SNR_THRESHOLD : # Too much noise drop detection.
                    continue
                elif abs(f_VrelRad) < VELOCITY_LOWER_THRESHOLD :
                    continue
                elif f_Range > DISTANCE_MAX_THRESHOLD : # need to do trig
                    continue
                elif f_Range < DISTANCE_MIN_THRESHOLD :
                    continue
            #    print(f"success")
                # move check prob to parser.cpp
                VrelRad      = f_VrelRad
                ElAng        = 0       #f_ElAng #Told to ignore by continental
                RangeVar     = f_RangeVar
                VrelRadVar   = f_VrelRadVar
                ElAngVar     = f_ElAngVar
                SNR          = f_SNR

                # VEEEERY SIMPLIFIED, probably will be more complex#accurate than this
                posX = f_Range
                posY = -1 * posX * math.tan(AzAng)  #Flip y axis
                posZ = posX * math.tan(ElAng)              

                tmpdata=[posX,posY,posZ,VrelRad,AzAng,ElAng,RCS,RangeVar,VrelRadVar,AzAngVar,ElAngVar,SNR]
            #    print(f"tmpdata:{tmpdata}")

                msg.Detections.extend(tmpdata)
            if detectionsInPacket!=0 and len(msg.Detections)!=0:
                print(f"msg.EventID:{msg.EventID},msg.TimeStamp:{msg.TimeStamp},msg.MeasurementCounter:{msg.MeasurementCounter},msg.Vambig:{msg.Vambig},msg.CenterFrequency:{msg.CenterFrequency},msg.Detections:{msg.Detections}")


            #    radarDetectionList.append([f_Range,f_VrelRad,f_AzAng0,f_AzAng1,f_ElAng,f_RCS0,f_RCS1,f_Prob0,f_Prob1,f_RangeVar,f_VrelRadVar,f_AzAngVar0,f_AzAngVar1,f_ElAngVar,f_Pdh0,f_SNR])
             #   radarDetectionList.append([f_SNR*0.1+11,f_Range*0.0045778,f_VrelRad*0.0045778,f_AzAng0*0.000095877,f_AzAng1*0.000095877,f_ElAng*0.000095877])
              #  radarDetectionList.append([f_Range*0.0045778, f_VrelRad*0.0045778, f_AzAng0*0.000095877*180#3.1415926,f_AzAng1*0.000095877*180#3.1415926,f_ElAng*0.000095877*180#3.1415926])

            if(HeaderID == 14417921 or HeaderID == 14417922):

                if(farScanTimeStamp > utcTimeStamp):
                    continue
                if(farScanTimeStamp < utcTimeStamp):

                    curFarIdx = (curFarIdx + 1) % 2

                    if curFarIdx == curNearIdx : #Indexes are the same
                        # We just got a new TS for both near & far, so we should publish the old buffer
                        publish = True

                    farScanTimeStamp = utcTimeStamp

                curGroup = PacketsBuffer[curFarIdx] #Tmp to make code easier to read
                # Load message into current buffer

                # print(f"curGroup.numFarPackets:{curGroup.numFarPackets},msg:{msg}")
            #     if loadRDIMessageFromPacket(curGroup.farPackets[curGroup.numFarPackets], msg) :
            #  #   if curGroup.farPackets.extend(msg) :
            #         curGroup.numFarPackets = curGroup.numFarPackets +1

                # if (publish) :
                #     err = publishPackets((curFarIdx+1)%2) #Publish the previous buffer idx
                #     publish = False
                # #    pthread_mutex_unlock(&Mutex)
                #     return err

            #    print(f"detectionsInPacket:{detectionsInPacket}")     

            else: # HeaderID == 14417923 || HeaderID == 14417924 || HeaderID == 14417924
                if(nearScanTimeStamp > utcTimeStamp):
                    continue
                if(nearScanTimeStamp < utcTimeStamp):
                    #NearFullScan is complete (all the packets related to far scan has been received) 

                    curNearIdx = (curNearIdx + 1) % 2

                    if curNearIdx == curFarIdx : #Indexes are the same
                        # We just got a new TS for both near & far, so we should publish the old buffer
                        publish = True
                    nearScanTimeStamp = utcTimeStamp 

                curGroup = PacketsBuffer[curFarIdx] #Tmp to make code easier to read
                # Load message into current buffer
            #     print(f"curGroup.numNearPackets:{curGroup.numNearPackets},msg:{msg}")
            #     if loadRDIMessageFromPacket(curGroup.nearPackets[curGroup.numNearPackets], msg) :
            #  #   if curGroup.nearPackets.extend(msg) :
            #         curGroup.numNearPackets = curGroup.numNearPackets +1
                
                # if (publish) :
                #     err = publishPackets((curFarIdx+1)%2) #Publish the previous buffer idx
                #     publish = False
                # #    pthread_mutex_unlock(&Mutex)
                #     return err

        else:
            print("Invalid header ID received from radar unit.")


sendStatus = False


PC_IP = "192.168.1.100"
RADAR_DESTINATION_PORTS = [31122, 31122, 31122, 31122, 31122, 31122, 31122]
RADAR_DESTINATION_IPS = ["225.0.0.1", "226.0.0.1", "227.0.0.1", "228.0.0.1", "229.0.0.1", "230.0.0.1", "231.0.0.1"]


socks = []
processes = []
#for i in range(1):
for i in range(len(RADAR_DESTINATION_PORTS)):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    sock.bind((RADAR_DESTINATION_IPS[i], RADAR_DESTINATION_PORTS[i]))
    sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(PC_IP))
    sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP,
			       socket.inet_aton(RADAR_DESTINATION_IPS[i]) + socket.inet_aton(PC_IP))
    socks.append(sock)
    processes.append(Process(target=receiveAndPackAndSendRadarData, args=(socks[-1], i)))
    processes[-1].start()

#for i in range(1):
for i in range(len(RADAR_DESTINATION_PORTS)):
    processes[i].join()


