#!/usr/bin/python

import rospy
from std_msgs.msg import Float64
from autoware_msgs.msg import ControlCommandStamped
from sd_control_msgs.msg import Control


class SdControl:

    def __init__(self):
        rospy.init_node("sd_control", anonymous=True)

        self.steer = None

        self.sd_control_pub = rospy.Publisher(
            "/sd_control", Control, queue_size=1
        )

        self.ctrl_cmd_sub = rospy.Subscriber(
            "/ctrl_cmd", ControlCommandStamped, self.callback_ctrl
        )

        self.throttle_sub = rospy.Subscriber(
            "/sd_throttle/control_effort", Float64, self.callback_throttle
        )

    def callback_ctrl(self, ctrl_cmd):
        self.steer = ctrl_cmd.cmd.steering_angle / 0.785398 * 100

    def callback_throttle(self, throttle):
        if self.steer is None:
            return

        msg = Control()
        msg.steer = self.steer
        msg.throttle = throttle.data
        self.sd_control_pub.publish(msg)

if __name__ == '__main__':
    try:
        sd_control = SdControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
