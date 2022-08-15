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
        self.rawData = [channel0, channel1, channel2, channel3, channel4, channel5]

    def talker(self, msg):
        pub = rospy.Publisher('ati_readings', msg, queue_size=10)
        rospy.init_node('ati_pub', anonymous=True)
        r = rospy.Rate(10)  # 10 Hz
        msg.name = 'ATI_loadcell_ID'
        msg.x = 0
        msg.y = 1
        msg.z = 2
        msg.mx = 3
        msg.my = 4
        msg.mz = 5

        while not rospy.is_shutdown():
            rospy.loginfo(ati)
            #print(msg.name, msg.x, msg.y, msg.mx, msg.my, msg.mz)
            pub.publish(ati)
            r.sleep()


if __name__ == '__main__':
    ati_ft = ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True)
    print(ati_ft.__str__())
    ati_ft.talker(ati)


