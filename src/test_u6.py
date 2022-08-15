#!/usr/bin/env python3
import rospy
from TAS_python_msg.msg import ati
import math
from datetime import datetime
import u6


# MAX_REQUESTS is the number of packets to be read.
MAX_REQUESTS = 75
# SCAN_FREQUENCY is the scan frequency of stream mode in Hz
SCAN_FREQUENCY = 5000
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

        # Read even channels for differential voltages
        channel0 = self.daq_device.getAIN(0)
        channel1 = self.daq_device.getAIN(2)
        channel2 = self.daq_device.getAIN(4)
        channel3 = self.daq_device.getAIN(6)
        channel4 = self.daq_device.getAIN(8)
        channel5 = self.daq_device.getAIN(10)
        rawData = [channel0, channel1, channel2, channel3, channel4, channel5]
        return rawData


    def talker(self):
        pub = rospy.Publisher('ati_readings', ati, queue_size=10)
        rospy.init_node('ati_pub', anonymous=True)
        r = rospy.Rate(10)  # 10 Hz

        msg = ati()
        msg.name = 'ATI_loadcell_ID'
        msg.x = data[0]
        msg.y = data[0]
        msg.z = data[0]
        msg.mx = data[0]
        msg.my = data[0]
        msg.mz = data[0]

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        #print(msg.name, msg.x, msg.y, msg.mx, msg.my, msg.mz)
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    ati = ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True)
    ati.talker()


