#!/usr/bin/env python3
import rospy
import os
from sensor_msgs.msg import JointState
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode
import numpy as np
import actionlib
from TAS_python_msg.msg import ati


class ATI_ft():

    def __init__(self):
        self.save_path = os.path.dirname(os.path.abspath(__file__)) + '/data/'  # path to save data
        self.print_force_torque = False
        self.counter = 0
        self.force_torque_array = np.empty((0, 1))

    def joint_trajectory_callback(self, data):

        data = data.goal.trajectory
        # rospy.loginfo(data.goal.trajectory)
        self.print_JointStates = True

    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('ati_sub', anonymous=True)
        rospy.Subscriber("/ati_readings", ati, self.ATI_loadcell)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def ATI_loadcell(self, ati):

        ati_id = ati.name
        rospy.loginfo(ati_id)
        fx = ati.x
        fy = ati.y
        fz = ati.z
        mx = ati.mx
        my = ati.my
        mz = ati.mz
        # force_array = np.append(force_array, joints)
        # np.savetxt(save_path + 'ati_forces.txt', force_array)
        # right now set to the max torque of the ati mini45 ft sensor
        # the end effector of the ur10 has a max torque of 56Nm
        if self.isViolation(ati):
            rospy.loginfo("Abort!!!\n" "Max force/torque violation!!!.")
            print(f'fx: {fx}, fy: {fy}, fz: {fz}, mx: {mx}, my: {my}, mz: {mz}')
            self.set_robot_to_mode(RobotMode.POWER_OFF)


        #print(f'fx: {fx}, fy: {fy}, fz: {fz}, mx: {mx}, my: {my}, mz: {mz}')
        rospy.loginfo(ati)

    def set_robot_to_mode(self, target_mode):
        timeout = rospy.Duration(5)
        set_mode_client = actionlib.SimpleActionClient('/ur_hardware_interface/set_mode', SetModeAction)
        if not set_mode_client.wait_for_server(timeout):
                fail(
                    "Could not reach set_mode action. Make sure that the driver is actually running."
                    " Msg: {}".format(err))
        goal = SetModeGoal()
        goal.target_robot_mode = target_mode
        goal.play_program = True # we use headless mode during tests
        # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
        goal.stop_program = True

        set_mode_client.send_goal(goal)
        set_mode_client.wait_for_result()
        return set_mode_client.get_result().success

    def isViolation(self, ati):
        fx = ati.x
        fy = ati.y
        fz = ati.z
        mx = ati.mx
        my = ati.my
        mz = ati.mz
        if fx >= 290.0 or fy >= 290.0 or fz >= 580.0 or mx >= 10.0 or my >= 10.0 or mz >= 10.0:
            return True
        elif fx <= -290.0 or fy <= -290.0 or fz <= -5.0 or mx <= -10.0 or my <= -10.0 or mz <= -10.0:
            return True
        return False

if __name__ == '__main__':
    load_cell = ATI_ft()
    load_cell.listener()

