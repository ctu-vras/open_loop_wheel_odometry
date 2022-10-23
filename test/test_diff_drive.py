#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for diff_drive_odometry"""

import math
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Pose, Twist, TwistStamped
from nav_msgs.msg import Odometry


pose_cov = [
    0.005, 0.005, 0.0, 0.0, 0.0, 0.0,
    0.005, 0.005, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2 * math.pi * math.pi, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 2 * math.pi * math.pi, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.5
]


twist_cov = [
    1e-2, 0, 0, 0, 0, 0,
    0, 1e-4, 0, 0, 0, 0,
    0, 0, 1e-1, 0, 0, 0,
    0, 0, 0, math.pi * math.pi, 0, 0,
    0, 0, 0, 0, math.pi * math.pi, 0,
    0, 0, 0, 0, 0, 0.5,
]


final_pose = Pose()
final_pose.position.x = 0.900316
final_pose.position.y = 0.372923
final_pose.orientation.z = 0.382683
final_pose.orientation.w = 0.923880


class DiffDrive(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(DiffDrive, self).__init__(*args, **kwargs)
        rospy.init_node("diff_drive", anonymous=True)
        self.msg = None

    def cb(self, msg):
        self.msg = msg

    def test_diff_drive(self):
        sub = rospy.Subscriber("odom_cmd_vel", Odometry, self.cb, queue_size=10)
        pub = rospy.Publisher("cmd_vel_out", Twist, queue_size=2)
        pub_reset = rospy.Publisher("reset", Twist, queue_size=2)

        for i in range(50):
            has_connections = True
            has_connections = has_connections and pub.get_num_connections() > 0
            has_connections = has_connections and pub_reset.get_num_connections() > 0
            if has_connections:
                break
            time.sleep(0.01)  # give publishers time to set up

        twist = Twist()
        twist.linear.x = 1
        twist.angular.z = math.pi / 4

        self.msg = None
        pub.publish(twist)
        time.sleep(1.0)
        self.assertIs(self.msg, None)

        pub.publish(twist)
        for i in range(50):
            if self.msg is not None:
                break
            time.sleep(0.01)
        self.assertIsNot(self.msg, None)

        assert isinstance(self.msg, Odometry)
        self.assertIsInstance(self.msg, Odometry)

        self.assertGreater(rospy.Time.now() + rospy.Duration(10), self.msg.header.stamp)
        self.assertLess(rospy.Time.now() - rospy.Duration(10), self.msg.header.stamp)
        self.assertEqual("odom_cmd_vel", self.msg.header.frame_id)
        self.assertEqual("base_link", self.msg.child_frame_id)

        self.assertEqual(twist, self.msg.twist.twist)
        for i in range(len(twist_cov)):
            self.assertAlmostEqual(twist_cov[i], self.msg.twist.covariance[i], delta=1e-6)

        self.assertAlmostEqual(final_pose.position.x, self.msg.pose.pose.position.x, delta=1e-1)
        self.assertAlmostEqual(final_pose.position.y, self.msg.pose.pose.position.y, delta=1e-1)
        self.assertAlmostEqual(final_pose.position.z, self.msg.pose.pose.position.z, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.x, self.msg.pose.pose.orientation.x, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.y, self.msg.pose.pose.orientation.y, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.z, self.msg.pose.pose.orientation.z, delta=1e-1)
        self.assertAlmostEqual(final_pose.orientation.w, self.msg.pose.pose.orientation.w, delta=1e-1)
        for i in range(len(pose_cov)):
            self.assertAlmostEqual(pose_cov[i], self.msg.pose.covariance[i], delta=1e-1)

        pub_reset.publish(Twist())
        time.sleep(0.1)

        self.msg = None
        pub.publish(twist)
        time.sleep(1.0)
        self.assertIs(self.msg, None)

        pub.publish(twist)
        for i in range(50):
            if self.msg is not None:
                break
            time.sleep(0.01)
        self.assertIsNot(self.msg, None)

        assert isinstance(self.msg, Odometry)
        self.assertIsInstance(self.msg, Odometry)

        self.assertGreater(rospy.Time.now() + rospy.Duration(10), self.msg.header.stamp)
        self.assertLess(rospy.Time.now() - rospy.Duration(10), self.msg.header.stamp)
        self.assertEqual("odom_cmd_vel", self.msg.header.frame_id)
        self.assertEqual("base_link", self.msg.child_frame_id)

        self.assertEqual(twist, self.msg.twist.twist)
        for i in range(len(twist_cov)):
            self.assertAlmostEqual(twist_cov[i], self.msg.twist.covariance[i], delta=1e-6)

        self.assertAlmostEqual(final_pose.position.x, self.msg.pose.pose.position.x, delta=1e-1)
        self.assertAlmostEqual(final_pose.position.y, self.msg.pose.pose.position.y, delta=1e-1)
        self.assertAlmostEqual(final_pose.position.z, self.msg.pose.pose.position.z, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.x, self.msg.pose.pose.orientation.x, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.y, self.msg.pose.pose.orientation.y, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.z, self.msg.pose.pose.orientation.z, delta=1e-1)
        self.assertAlmostEqual(final_pose.orientation.w, self.msg.pose.pose.orientation.w, delta=1e-1)
        for i in range(len(pose_cov)):
            self.assertAlmostEqual(pose_cov[i], self.msg.pose.covariance[i], delta=1e-1)

    def test_diff_drive_stamped(self):
        sub_stamped = rospy.Subscriber("odom_cmd_vel_stamped", Odometry, self.cb, queue_size=10)
        pub_stamped = rospy.Publisher("cmd_vel_out_stamped", TwistStamped, queue_size=2)
        pub_reset = rospy.Publisher("reset", Twist, queue_size=2)

        for i in range(50):
            has_connections = True
            has_connections = has_connections and pub_stamped.get_num_connections() > 0
            has_connections = has_connections and pub_reset.get_num_connections() > 0
            if has_connections:
                break
            time.sleep(0.01)  # give publishers time to set up

        twist = Twist()
        twist.linear.x = 1
        twist.angular.z = math.pi / 4
        twist_stamped = TwistStamped()
        twist_stamped.twist = twist

        self.msg = None
        twist_stamped.header.stamp = rospy.Time(1)
        pub_stamped.publish(twist_stamped)
        time.sleep(0.1)
        self.assertIs(self.msg, None)

        twist_stamped.header.stamp = rospy.Time(2)
        pub_stamped.publish(twist_stamped)
        for i in range(50):
            if self.msg is not None:
                break
            time.sleep(0.01)
        self.assertIsNot(self.msg, None)

        assert isinstance(self.msg, Odometry)
        self.assertIsInstance(self.msg, Odometry)

        self.assertEqual(rospy.Time(2), self.msg.header.stamp)
        self.assertEqual("odom_cmd_vel", self.msg.header.frame_id)
        self.assertEqual("base_link", self.msg.child_frame_id)

        self.assertEqual(twist, self.msg.twist.twist)
        for i in range(len(twist_cov)):
            self.assertAlmostEqual(twist_cov[i], self.msg.twist.covariance[i], delta=1e-6)

        self.assertAlmostEqual(final_pose.position.x, self.msg.pose.pose.position.x, delta=1e-6)
        self.assertAlmostEqual(final_pose.position.y, self.msg.pose.pose.position.y, delta=1e-6)
        self.assertAlmostEqual(final_pose.position.z, self.msg.pose.pose.position.z, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.x, self.msg.pose.pose.orientation.x, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.y, self.msg.pose.pose.orientation.y, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.z, self.msg.pose.pose.orientation.z, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.w, self.msg.pose.pose.orientation.w, delta=1e-6)
        for i in range(len(pose_cov)):
            self.assertAlmostEqual(pose_cov[i], self.msg.pose.covariance[i], delta=1e-3)

        pub_reset.publish(Twist())
        time.sleep(0.1)

        self.msg = None
        twist_stamped.header.stamp = rospy.Time(1)
        pub_stamped.publish(twist_stamped)
        time.sleep(0.1)
        self.assertIs(self.msg, None)

        twist_stamped.header.stamp = rospy.Time(2)
        pub_stamped.publish(twist_stamped)
        for i in range(50):
            if self.msg is not None:
                break
            time.sleep(0.01)
        self.assertIsNot(self.msg, None)

        assert isinstance(self.msg, Odometry)
        self.assertIsInstance(self.msg, Odometry)

        self.assertEqual(rospy.Time(2), self.msg.header.stamp)
        self.assertEqual("odom_cmd_vel", self.msg.header.frame_id)
        self.assertEqual("base_link", self.msg.child_frame_id)

        self.assertEqual(twist, self.msg.twist.twist)
        for i in range(len(twist_cov)):
            self.assertAlmostEqual(twist_cov[i], self.msg.twist.covariance[i], delta=1e-6)

        self.assertAlmostEqual(final_pose.position.x, self.msg.pose.pose.position.x, delta=1e-6)
        self.assertAlmostEqual(final_pose.position.y, self.msg.pose.pose.position.y, delta=1e-6)
        self.assertAlmostEqual(final_pose.position.z, self.msg.pose.pose.position.z, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.x, self.msg.pose.pose.orientation.x, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.y, self.msg.pose.pose.orientation.y, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.z, self.msg.pose.pose.orientation.z, delta=1e-6)
        self.assertAlmostEqual(final_pose.orientation.w, self.msg.pose.pose.orientation.w, delta=1e-6)
        for i in range(len(pose_cov)):
            self.assertAlmostEqual(pose_cov[i], self.msg.pose.covariance[i], delta=1e-3)


if __name__ == '__main__':
    rostest.rosrun("open_loop_wheel_odometry", "diff_drive", DiffDrive)
