#!/usr/bin/env python3

from __future__ import division
import rospy
import numpy as np
import picamera.array
from pidrone_pkg.msg import State
from geometry_msgs.msg import TwistStamped
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Empty

from duckietown.dtros import DTROS, NodeType



class AnalyzeFlow(picamera.array.PiMotionAnalysis):
    ''' A class used for real-time motion analysis of optical flow vectors to
    obtain the planar and yaw velocities of the drone.

    For more information, read the following:
    http://picamera.readthedocs.io/en/release-1.10/api_array.html#picamera.array.PiMotionAnalysis
    '''

    def setup(self, camera_wh):
        ''' Initialize the instance variables '''

        # flow variables
        self.max_flow = camera_wh[0] / 16.0 * camera_wh[1] / 16.0 * 2**7
        self.flow_scale = .165
        self.flow_coeff = 100 * self.flow_scale / self.max_flow # (multiply by 100 for cm to m conversion)

        self.altitude = 0.0

        # ROS setup:
        ############
        # Publisher:
        self.twistpub = rospy.Publisher('/pidrone/picamera/twist', TwistStamped, queue_size=1)
        # Subscriber:
        rospy.Subscriber("/pidrone/state", State, self.state_callback)

    def analyse(self, a):
        ''' Analyze the frame, calculate the motion vectors, and publish the
        twist message. This is implicitly called by the

        a : an array of the incoming motion data that is provided by the
            PiMotionAnalysis api
        '''
        # signed 1-byte values
        x = a['x']
        y = a['y']

        # calculate the planar and yaw motions
        x_motion = np.sum(x) * self.flow_coeff * self.altitude
        y_motion = np.sum(y) * self.flow_coeff * self.altitude
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.twist.linear.x = x_motion
        twist_msg.twist.linear.y = - y_motion

        # Update and publish the twist message
        self.twistpub.publish(twist_msg)


    def state_callback(self, msg):
        """
        Store z position (altitude) reading from State
        """
        self.altitude = msg.pose_with_covariance.pose.position.z
        
class OpticalFlowNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(OpticalFlowNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )


        # subscribers
        self._sub_mv = rospy.Subscriber('~motion_vectors', H264MotionVectors, self.motion_cb, queue_size=1)
        self._sub_tof = rospy.Subscriber('~tof', Range, self.tof_cb, queue_size=1)



    def motion_cb(self, msg):
        print(msg)

    def tof_cb(self, msg):
        """
        The Time-of-Flight sensor gives the distance to the closest object below the robot.

        Args:
            msg:  the message from the Time-of-Flight sensor

        """
        print("range: %d", range)
        range = msg.range
        if range > msg.max_range:
            if self._last_range == 0:
                return
            # out-of-range, assume last range
            range = self._last_range
        # keep track of last range
        self._last_range = range
        # offset the range by the distance between the sensor frame and the footprint frame
        range = max(range - self._h_offset, 0.0)
        # compute altitude from range and angle
        altitude = range * np.cos(self._angle)
        # publish message
        msg = Range(
            header=msg.header,
            radiation_type=msg.radiation_type,
            field_of_view=msg.field_of_view,
            min_range=msg.min_range,
            max_range=msg.max_range,
            range=altitude
        )
        self._pub.publish(msg)
        # update heartbeat
        self._heartbeat.publish(Empty())

    
def main():
    print "hello"
    optical_flow_node = OpticalFlowNode("optical_flow_node")
    rospy.spin()

if __name__ == '__main__':
    main()
