#!/usr/bin/env python3
import rospy
from TAS_python_msg.msg import ati
from datetime import datetime
import u6
from LabJackPython import NullHandleException
import time
import numpy


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
        self.rawData = []
        self.forces = []

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
        #print(self.rawData)

    def convertingRawData(self):
        bias = [-0.28026725005202024, -0.5628451798374954, 0.36969605272634, -0.021619200849272602, -0.1790193724709752, 0.2219047680659969]
        userAxis = [[-1.548979761, 0.1884502090, 5.8967571490, -48.38976343, -3.046532067, 46.300900810],
                    [-7.146532849, 55.028037290, 0.4930935770, -28.03857447, 4.4214399430, -26.78444170],
                    [69.408622200, 1.6678919470, 69.830354820, 2.8068233740, 65.972226370, 2.3081625690],
                    [-0.061111397, 0.3890235240, -1.113368181, -0.245195740, 1.1272834260, -0.147965939],
                    [1.2734045750, 0.0406643920, -0.695352495, 0.3113292350, -0.587150713, -0.349685365],
                    [0.1045067140, -0.708440304, 0.0849889820, -0.710842342, 0.0348277400, -0.683977902]]
        offSetCorrection = self.rawData - numpy.transpose(bias)
        self.forces = numpy.dot(userAxis, numpy.transpose(offSetCorrection))
        #print(self.rawData)
        #print(offSetCorrection)
        #print(f'forces {self.forces}')

    def isConnected(self):
        if self.daq_device is None:
            global DAQ_STATE
            DAQ_STATE = False
            raise NullHandleException()

    def talker(self):
        pub = rospy.Publisher('ati_readings', ati, queue_size=10)
        rospy.init_node('ati_pub', anonymous=True)
        r = rospy.Rate(100)  # 100 Hz
        msg = ati()
        msg.name = 'ATI_FT35016'
        msg.x = self.forces[0]
        msg.y = self.forces[1]
        msg.z = self.forces[2]
        msg.mx = self.forces[3]
        msg.my = self.forces[4]
        msg.mz = self.forces[5]

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
    timeout = time.time() + 60

    # time.time() < timeout # running for specify amount of time
    while not rospy.is_shutdown():
        ati_ft.getAnalogChannels()
        ati_ft.convertingRawData()
        ati_ft.talker()
        if not DAQ_STATE:
            print('DAQ is disconnected!')
            break

    stop = datetime.now()
    print(f"Stop time is {stop}")

