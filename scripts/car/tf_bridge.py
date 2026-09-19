#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""把 Gazebo 插件发布的坐标系，桥接到本项目使用的“带命名空间”坐标系。

背景
----
多机仿真里每台车的 TF 必须带前缀（robot_red1/odom、robot_red1/base_footprint …），
否则 6 台车会互相覆盖同一组坐标系，amcl 也就找不到
map → robot_red1/odom → robot_red1/base_footprint 这条链，
表现就是 amcl 不出 amcl_pose、move_base 一直走 recovery 原地自转。

而 turtlebot3 原版 URDF 里的 gazebo 插件用的是 odom / base_footprint / base_scan
这类**不带前缀**的名字（是否自动补前缀取决于插件版本与 robotNamespace 设置）。

本节点做两件事：
  1) 读 /<ns>/odom 的 header.frame_id / child_frame_id 和 /<ns>/scan 的 frame_id，
     原样打印出来，方便确认插件实际用的是哪套名字；
  2) 只在实际名字不等于期望的带前缀名字时，补一条 identity 静态变换把它们接起来：
       odom 没前缀  -> 补 (robot_xx/odom  -> odom)
       base 没前缀  -> 补 (base_footprint -> robot_xx/base_footprint)
       laser 没前缀 -> 补 (robot_xx/base_scan -> base_scan)
     名字本来就带前缀时什么都不发，避免人为制造 TF 环路。

节点运行在每台车的命名空间下（launch 里放在 <group ns="robot_xxx"> 内，和 amcl 一起启动）。
"""

import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class TfBridge(object):
    def __init__(self):
        self.ns = rospy.get_namespace().strip("/")
        self.expected_odom = "%s/odom" % self.ns
        self.expected_base = "%s/base_footprint" % self.ns
        self.expected_laser = "%s/base_scan" % self.ns

        self._pub = rospy.Publisher("/tf_static", TFMessage, queue_size=10, latch=True)
        self._published = set()
        self._odom_frames = None
        self._scan_frame = None

        self._odom_sub = rospy.Subscriber(
            "/%s/odom" % self.ns, Odometry, self._odom_cb, queue_size=1)
        self._scan_sub = rospy.Subscriber(
            "/%s/scan" % self.ns, LaserScan, self._scan_cb, queue_size=1)

        rospy.loginfo(
            "[tf_bridge/%s] 等待 /%s/odom 与 /%s/scan，用来判断坐标系前缀是否匹配",
            self.ns, self.ns, self.ns,
        )

    @staticmethod
    def _clean(name):
        return str(name or "").strip().strip("/")

    def _odom_cb(self, msg):
        frames = (self._clean(msg.header.frame_id), self._clean(msg.child_frame_id))
        if frames == self._odom_frames:
            return
        self._odom_frames = frames
        self._publish_bridges()

    def _scan_cb(self, msg):
        frame = self._clean(msg.header.frame_id)
        if frame == self._scan_frame:
            return
        self._scan_frame = frame
        self._publish_bridges()

    def _publish_bridges(self):
        if self._odom_frames is None:
            return
        actual_odom, actual_base = self._odom_frames
        actual_scan = self._scan_frame

        rospy.loginfo(
            "[tf_bridge/%s] 实际坐标系: odom=%r base=%r scan=%r (期望 %r / %r / %r)",
            self.ns, actual_odom, actual_base, actual_scan,
            self.expected_odom, self.expected_base, self.expected_laser,
        )

        pairs = []
        if actual_odom and actual_odom != self.expected_odom:
            # 让带前缀的 odom 成为不带前缀 odom 的父节点：robot_xx/odom -> odom
            pairs.append((self.expected_odom, actual_odom))
        if actual_base and actual_base != self.expected_base:
            # 让不带前缀的 base 成为带前缀 base 的父节点：base_footprint -> robot_xx/base_footprint
            pairs.append((actual_base, self.expected_base))
        if actual_scan and actual_scan != self.expected_laser:
            # 让激光报文里的坐标系挂到带前缀的 laser link 下：robot_xx/base_scan -> base_scan
            pairs.append((self.expected_laser, actual_scan))

        new_pairs = [pair for pair in pairs if pair not in self._published]
        if not new_pairs:
            return

        message = TFMessage()
        for parent, child in new_pairs:
            message.transforms.append(self._identity(parent, child))
            self._published.add((parent, child))
            rospy.logwarn(
                "[tf_bridge/%s] 插件坐标系缺前缀，补一条静态变换: %s -> %s",
                self.ns, parent, child,
            )
        self._pub.publish(message)

    @staticmethod
    def _identity(parent, child):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        return transform


def main():
    rospy.init_node("tf_bridge")
    TfBridge()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
