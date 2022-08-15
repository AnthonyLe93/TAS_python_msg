import math
from datetime import datetime
import u6

class ATI_readings:
    # using labjack u6
    daq_device = u6.U6()

    def __init__(self, **kwargs):
        self.resolutionIndex = kwargs['resolutionIndex']
        self.gainIndex = kwargs['gainIndex']
        self.settlingFactor = kwargs['settlingFactor']
        self.differential = kwargs['differential']

    def __str__(self):
        printOut = '''
        Resolution Index: {} 
        Gain Index: {} 
        Settlingfactor: {} 
        Differential: {}
        '''

        return printOut.format(self.resolutionIndex, self.gainIndex, self.settlingFactor, self.differential)



ati = ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True)
print(ati)

