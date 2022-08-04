#! /usr/bin/env python3
import rospy
import datetime

from ur_robot_driver.msg import ATI_mini45


def talker():
    pub = rospy.Publisher('ATI_readings', ATI_mini45, queue_size=10)
    rospy.init_node('ATI_pub', anonymous=True)
    r = rospy.Rate(10) # 10Hz

    msg = ATI_mini45()

    msg.name.data = 'ATI'

    msg.force.x = 10
    msg.force.y = 10
    msg.force.z = 15
    msg.torque.x = 2
    msg.torque.y = 3
    msg.torque.z = 4

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass




