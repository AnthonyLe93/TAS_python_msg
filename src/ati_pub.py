#!/usr/bin/env python3
import rospy
from TAS_python_msg.msg import ati


def talker():
    pub = rospy.Publisher('ATI_readings', ati, queue_size=10)
    rospy.init_node('ATI')
    r = rospy.Rate(10)  # 10 Hz

    msg = ati()
    msg.name = 'ATI_loadcell_ID'
    msg.x = 1.0
    msg.y = 2.0
    msg.y = 3.0
    msg.mx = 4.0
    msg.my = 5.0
    msg.mz = 6.0

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        #print(msg.name, msg.x, msg.y, msg.mx, msg.my, msg.mz)
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



