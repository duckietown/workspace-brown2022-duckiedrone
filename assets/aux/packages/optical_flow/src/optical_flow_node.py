#!/usr/bin/env python3

from __future__ import division
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from duckietown_msgs.msg import H264MotionVectors
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty

from duckietown.dtros import DTROS, NodeType


class OpticalFlowNode(DTROS):
    """
    Subscribe to the optical flow vectors and publish linear velocity as a Twist message.
    """
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(OpticalFlowNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self.twistpub = rospy.Publisher('/pidrone/picamera/twist', TwistStamped, queue_size=1)
        # flow variables
        camera_wh = (320, 240)        
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.flow_scale = .165
        self.flow_coeff = 100 * self.flow_scale / self.max_flow # (multiply by 100 for cm to m conversion)

        self.altitude = 0.0


        # subscribers
        self._sub_mv = rospy.Subscriber('/cosmo/camera_node/motion_vectors', H264MotionVectors, self.motion_cb, queue_size=1)
        self._sub_tof = rospy.Subscriber('/cosmo/altitude_node/altitude', Range, self.altitude_cb, queue_size=1)



    def motion_cb(self, msg):
        ''' Average the motion vectors and publish the
        twist message. 
        '''
        # signed 1-byte values
        x = msg.x
        y = msg.y

        # calculate the planar and yaw motions
        x_motion = np.sum(x) * self.flow_coeff * self.altitude
        y_motion = np.sum(y) * self.flow_coeff * self.altitude
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = x_motion
        twist_msg.twist.linear.y = - y_motion

        # Update and publish the twist message
        self.twistpub.publish(twist_msg)


    def altitude_cb(self, msg):
        """
        The altitude of the robot
        Args:
            msg:  the message publishing the altitude

        """
        self.altitude = msg.range
    
def main():
    optical_flow_node = OpticalFlowNode("optical_flow_node")
    rospy.spin()

if __name__ == '__main__':
    main()
