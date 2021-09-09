#from _typeshed import Self
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
from sensor_msgs.msg import PointCloud2, PointField
#import tf


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

class PointCloud2:
    def __init__(self):
        self 
        

# class radarFarFullScanMessage:
#     def __init__(self):
#         farScanTimeStamp = ""
#         self.far = []

# class radarNearFullScanMessage:
#     def __init__(self):
#         nearScanTimeStamp = ""
#         self.near = []

class Radar_Packet_Msg :
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
        self.FarPackets = [[]]
        self.NearPackets  = [[]]
        self.numFarPackets  = 0
        self.numNearPackets = 0
        self.curNearBufId = 0
        self.curFarBufId = 0
    
    def append_far(self,packet):
        self.FarPackets[self.curFarBufId].append(packet)
        print(f"len_FarPackets:{len(self.FarPackets)}")
        self.numFarPackets += 1

    def append_near(self,packet):
        self.NearPackets[self.curNearBufId].append(packet)
        print(f"len_NearPackets:{len(self.NearPackets)}")
        self.numNearPackets += 1   

    def far_is_ready(self):
        self.FarPackets.append([])
        self.curFarBufId += 1

    def near_is_ready(self):
        self.NearPackets.append([])
        self.curNearBufId += 1

    def far_del(self):
        print(f"before del farPackets[0] is {self.curFarBufId}")
        del self.FarPackets[0]
        self.curFarBufId -= 1
        print(f"after del farPackets[0] is {self.curFarBufId}")

    def near_del(self):
        print(f"before del farPackets[0] is {self.curNearBufId}")
        del self.NearPackets[0]
        self.curNearBufId -= 1
        print(f"after del farPackets[0] is {self.curNearBufId}")



# def loadRDIMessageFromPacket(newMsg, oldMsg):

#     newMsg.EventID                    = oldMsg.EventID
#     newMsg.TimeStamp                  = oldMsg.TimeStamp
#     newMsg.MeasurementCounter         = oldMsg.MeasurementCounter
#     newMsg.Vambig                     = oldMsg.Vambig
#     newMsg.CenterFrequency            = oldMsg.CenterFrequency
#     newMsg.Detections = []

#     for  i in range(len(oldMsg.Detections)) :

#         tmpdata = Data()

#         ## VEEEERY SIMPLIFIED, probably will be more complex/accurate than this
#         tmpdata.posX = oldMsg.Detections[i].posX
#         tmpdata.posY = -1 * tmpdata.posX * math.tan(tmpdata.AzAng)  ##Flip y axis
#         tmpdata.posZ = tmpdata.posX * math.tan(tmpdata.ElAng)
#         ## move check prob to parser.cpp
#         tmpdata.AzAng        = oldMsg.Detections[i].AzAng
#         tmpdata.RCS          = oldMsg.Detections[i].RCS
#         tmpdata.AzAngVar     = oldMsg.Detections[i].AzAngVar
#         tmpdata.VrelRad      = oldMsg.Detections[i].VrelRad
#         tmpdata.ElAng        = 0 ##oldMsg.Detections[i].f_ElAng ##Told to ignore by continental
#         tmpdata.RangeVar     = oldMsg.Detections[i].RangeVar
#         tmpdata.VrelRadVar   = oldMsg.Detections[i].VrelRadVar
#         tmpdata.ElAngVar     = oldMsg.Detections[i].ElAngVar
#         tmpdata.SNR          = oldMsg.Detections[i].SNR

#         newMsg.Detections.append(tmpdata)

#     ## Return True if there is at least one detection in newMsg after filtering
#     print(f"copy to tmp grouppacket, len(newMsg.Detections):{len(newMsg.Detections)}")
#     return len(newMsg.Detections)

def radar2pointcloud2(group) :
  #  tf_br = tf.TransformBroadcaster()
    
    pc = PointCloud2()
    pc.header = group.header
    pc.header.frame_id = "radar_fixed"
    pc.height = 1
    pc.width = 0

    pf = PointField()

    ##Set x field type
    pf.name='x'
    pf.offset=0
    pf.datatype=7
    pf.count=1
    pc.fields.append(pf)

    pf.name='y' ##Same except name & offset, reuse
    pf.offset=4
    pc.fields.append(pf)

    pf.name='z'
    pf.offset=8
    pc.fields.append(pf)

    pf.name="intensity" ##Green = small (low RCS), Red = large (high RCS)
    pf.offset=12
    pc.fields.append(pf)

    pc.is_bigendian = False ##All computer tested on are little endian
    pc.point_step = 16 ##4 bytes for x, 4 bytes for y, 4 bytes for z, 4 bytes for intensity
    pc.is_dense = True

    for i in range( group.size()):
        for j in  range(group[i].Detections.size()):
            pc.width += 1
            tmp = group[i].Detections[j].posX ##My computer is little endian (lsb @ lowest index)
            for k in range(4):
                pc.data.append(tmp[k])
            
            tmp = group[i].Detections[j].posY
            for k in range(4):
                pc.data.append(tmp[k])
            
            tmp = group[i].Detections[j].posZ
            for k in range(4):
                pc.data.append(tmp[k])
            
            ##Map from -100/+100 to 0/+100, lower span & more color changes
            intensity = (group[i].Detections[j].RCS / 2.0) + 50.0
            tmp = intensity ##Turn float32 into 4 bytes
            for k in range(4):
                pc.data.append(tmp[k])
            
    pc.row_step = pc.point_step * pc.width
#     leftFOVLine.header.stamp = rightFOVline.header.stamp = pc.header.stamp

#     tf_br.sendTransform(tf.StampedTransform(fixed, ros.Time.now(), "/base_link", BASE_FRAME))

#     pcData.publish(pc)

#     marker_pub.publish(leftFOVLine)
#     marker_pub.publish(rightFOVline)

#     return

# curTimeStamp=0
# def radarCallback(msg):
#     if curTimeStamp == 0 :
#         curTimeStamp = msg.TimeStamp
#         packets.append(msg)
#     elif (curTimeStamp != msg.TimeStamp) :
#         curTimeStamp = msg.TimeStamp
#         newGroup = RadarPacket()
#         newGroup = packets ##Make a local copy
#         packets.clear()
#         packets.append(msg)
#         packetCloudGenerator(newGroup)
#     else : ## Time Stamps are the same
#         packets.append(msg))
    
#     return


# def configureFOVlines(self):
#     leftFOVLine.header.frame_id = rightFOVline.header.frame_id = BASE_FRAME
#     ##Configure timestamp at each publish
#     leftFOVLine.ns = rightFOVline.ns = "points_and_lines"
#     leftFOVLine.action = rightFOVline.action = visualization_msgs.Marker.ADD
#     leftFOVLine.pose.orientation.w = rightFOVline.pose.orientation.w = 1.0

#     leftFOVLine.id = 1
#     rightFOVline.id = 2

#     leftFOVLine.type = rightFOVline.type = visualization_msgs.Marker.LINE_STRIP

#     leftFOVLine.scale.x = rightFOVline.scale.x = 0.1
#     leftFOVLine.color.b = rightFOVline.color.b = 1.0
#     leftFOVLine.color.a = rightFOVline.color.a = 1.0

#     geometry_msgs.Point p
#     p.x = p.y = p.z = 0 ##Create origin point
#     rightFOVline.points.push_back(p)
#     leftFOVLine.points.push_back(p)

#     p.x = MAX_DIST ##Create farthest point
#     p.y = p.x * tan(FOV_ANGLE * M_PI / 180.0)
#     rightFOVline.points.push_back(p)
#     p.y *= -1
#     leftFOVLine.points.push_back(p)


def receiveAndPackAndSendRadarData(sock, index):
    nearScanTimeStamp = 0
    farScanTimeStamp  = 0
    frame_id = 0

    radargroup = PacketGroup()
    curGroup = PacketGroup()

    msg = Radar_Packet_Msg()  

    Far_is_Ready = False
    Near_is_Ready = False
     
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

            msg.EventID                    = HeaderID
            msg.TimeStamp                  = utcTimeStamp
            msg.MeasurementCounter         = mesurmentCounter
            msg.Vambig                     = vAmbig
            msg.CenterFrequency            = centerFrequency
            msg.Detections = []

            # print(f"detectionsInPacket:{detectionsInPacket}")
          #  radarDetectionList = []
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


                ####################### raw data processing to pointcloud2 data ############################

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


                #Filter through error flags in packet
                if f_Pdh0 and NEAR_PROB_MASK :
                    continue
                elif f_Pdh0 and INFERENCE_PROB_MASK :
                    continue
                elif f_Pdh0 and SIDELOBE_PROB_MASK : 
                    continue

                if f_Prob0 >= f_Prob1 :
                    AzAng        = f_AzAng0
                    RCS          = f_RCS0
                    AzAngVar     = f_AzAngVar0
                else:
                    AzAng        = f_AzAng1
                    RCS          = f_RCS1
                    AzAngVar     = f_AzAngVar1

                # Figure out an SNR threshold value that actually works here.
                if f_SNR < SNR_THRESHOLD : # Too much noise drop detection.
                    continue
                elif abs(f_VrelRad) < VELOCITY_LOWER_THRESHOLD :
                    continue
                elif f_Range > DISTANCE_MAX_THRESHOLD : # need to do trig
                    continue
                elif f_Range < DISTANCE_MIN_THRESHOLD :
                    continue

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
                msg.Detections.append(tmpdata)

            # if detectionsInPacket!=0 and len(msg.Detections)!=0:
            #    print(f"msg.EventID:{msg.EventID},msg.TimeStamp:{msg.TimeStamp},msg.MeasurementCounter:{msg.MeasurementCounter},msg.Vambig:{msg.Vambig},msg.CenterFrequency:{msg.CenterFrequency},msg.Detections:{msg.Detections}")
                
                # #TODO
                # pc_msg = PointCloud2() 
                # pc_msg = radar2pointcloud2(msg)


            ######################save and publish pointcloud2 data########################

           # add Far0,Far1 data into groupPacket
            if(HeaderID == 14417921 or HeaderID == 14417922):
                if (farScanTimeStamp == 0): ##Init Case
                    farScanTimeStamp = utcTimeStamp
                elif (farScanTimeStamp != utcTimeStamp) :##New timestamp
                    # old farframe is ready. append a new empty list to save new frame
                    curGroup.far_is_ready()
                    Far_is_Ready = True

                    ##old far frame and near frame are both ready, pulish them together
                    if Near_is_Ready:
                        ## We just got a new TS for both near & far, so we should publish the old buffer
                        # TODO:publish far and near and del old farpacket[0] and nearpacket[0]
                        # publishPackets() 
                        curGroup.far_del
                        curGroup.near_del
                        Far_is_Ready = False
                        Near_is_Ready = False

                    farScanTimeStamp = utcTimeStamp
                curGroup.append_far(msg)
   
            else: # HeaderID == 14417923 || HeaderID == 14417924 || HeaderID == 14417924
                if nearScanTimeStamp == 0:
                    continue
                if nearScanTimeStamp < utcTimeStamp:
                    # old nearframe is ready. append a new empty list to save new frame
                    curGroup.Near_is_ready()
                    Near_is_Ready = True

                    ##old far frame and near frame are both ready, pulish them together
                    if Far_is_Ready: 
                        ## We just got a new TS for both near & far, so we should publish the old buffer
                        # TODO:publish far and near and del old farpacket[0] and nearpacket[0]
                        # publishPackets() 
                        curGroup.far_del
                        curGroup.near_del
                        Far_is_Ready = False
                        Near_is_Ready = False

                    nearScanTimeStamp = utcTimeStamp 
                curGroup.append_near(msg)

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


