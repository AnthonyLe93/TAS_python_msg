#!/usr/bin/env python3
import rospy
from TAS_python_msg.msg import ati
import math
from datetime import datetime
import u6
from LabJackPython import NullHandleException
import time

# MAX_REQUESTS is the number of packets to be read.
MAX_REQUESTS = 75
# SCAN_FREQUENCY is the scan frequency of stream mode in Hz
SCAN_FREQUENCY = 5000
DAQ_STATE = True # make sure daq is connected

class ATI_readings:
    # using labjack u6

    def __init__(self, **kwargs):
        self.resolutionIndex = kwargs['resolutionIndex']
        self.gainIndex = kwargs['gainIndex']
        self.settlingFactor = kwargs['settlingFactor']
        self.differential = kwargs['differential']
        self.daq_device = u6.U6()
        self.daq_device.getCalibrationData()
        '''self.daq_device.streamConfig(NumChannels=6, ChannelNumbers=[0, 2, 4, 6, 8, 10],\
                                    ChannelOptions=[0, 0, 0, 0, 0, 0], SettlingFactor=self.settlingFactor,
                                    ResolutionIndex=self.resolutionIndex, ScanFrequency=SCAN_FREQUENCY)'''

        self.rawData = []



    def __str__(self):
        printOut = '''
        Resolution Index: {} 
        Gain Index: {} 
        Settlingfactor: {} 
        Differential: {}
        '''

        return printOut.format(self.resolutionIndex, self.gainIndex, self.settlingFactor, self.differential)

    def getAnalogChannels(self):

        self.isConnected()
        # Read even channels for differential voltages
        channel0 = self.daq_device.getAIN(0)
        channel1 = self.daq_device.getAIN(2)
        channel2 = self.daq_device.getAIN(4)
        channel3 = self.daq_device.getAIN(6)
        channel4 = self.daq_device.getAIN(8)
        channel5 = self.daq_device.getAIN(10)
        self.rawData = [channel0, channel1, channel2, channel3, channel4, channel5]
        print(self.rawData)

    def isConnected(self):
        if self.daq_device is None:
            global DAQ_STATE
            DAQ_STATE = False
            raise NullHandleException()

    def talker(self):
        pub = rospy.Publisher('ati_readings', ati, queue_size=10)
        rospy.init_node('ati_pub', anonymous=True)
        r = rospy.Rate(10)  # 10 Hz
        msg = ati()
        msg.name = 'ATI_loadcell_ID'
        msg.x = self.rawData[0]
        msg.y = self.rawData[1]
        msg.z = self.rawData[2]
        msg.mx = self.rawData[3]
        msg.my = self.rawData[4]
        msg.mz = self.rawData[5]

        #while not rospy.is_shutdown():
        rospy.loginfo(msg)
        #print(msg.name, msg.x, msg.y, msg.mx, msg.my, msg.mz)
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    ati_ft = ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True)
    print(ati_ft.__str__())
    start = datetime.now()
    print(f"Start time is {start}")
    timeout = time.time() + 30

    while time.time() < timeout:
        ati_ft.getAnalogChannels()
        ati_ft.talker()
        if not DAQ_STATE:
            print('DAQ is disconnected!')
            break

    stop = datetime.now()
    print(f"Stop time is {stop}")

