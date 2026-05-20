#!/usr/bin/env python3
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from rosplane_msgs.msg import Waypoint
from visualization_msgs.msg import Marker


WAYPOINT_SCALE = 5.0
TEXT_SCALE = 15.0
LINE_WIDTH = 3.0


class WaypointMarkerPublisher(Node):
    def __init__(self):
        super().__init__("voloxide_rosplane_waypoint_markers")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(Marker, "rviz/waypoint", qos)
        self.subscription = self.create_subscription(
            Waypoint, "waypoint_path", self.on_waypoint, qos
        )
        self.points = []
        self.next_id = 0
        self.clear_markers()

    def on_waypoint(self, waypoint):
        if waypoint.clear_wp_list:
            self.clear_markers()
            return

        now = self.get_clock().now().to_msg()
        point = Point(x=float(waypoint.w[0]), y=float(waypoint.w[1]), z=float(waypoint.w[2]))
        self.points.append(point)

        sphere = Marker()
        sphere.header.stamp = now
        sphere.header.frame_id = "NED"
        sphere.ns = "wp"
        sphere.id = self.next_id
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = WAYPOINT_SCALE
        sphere.scale.y = WAYPOINT_SCALE
        sphere.scale.z = WAYPOINT_SCALE
        sphere.color.r = 1.0
        sphere.color.a = 1.0

        text = Marker()
        text.header.stamp = now
        text.header.frame_id = "NED"
        text.ns = "text"
        text.id = self.next_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = point.x
        text.pose.position.y = point.y
        text.pose.position.z = point.z - WAYPOINT_SCALE - 1.0
        text.pose.orientation.w = 1.0
        text.scale.z = TEXT_SCALE
        text.color.a = 1.0
        text.text = str(self.next_id)

        line = Marker()
        line.header.stamp = now
        line.header.frame_id = "NED"
        line.ns = "wp_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = LINE_WIDTH
        line.color.g = 1.0
        line.color.a = 1.0
        line.points = list(self.points)

        self.publisher.publish(sphere)
        self.publisher.publish(text)
        self.publisher.publish(line)
        self.next_id += 1

    def clear_markers(self):
        self.points.clear()
        self.next_id = 0
        for namespace in ("wp", "text", "wp_path"):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "NED"
            marker.ns = namespace
            marker.action = Marker.DELETEALL
            self.publisher.publish(marker)


def main():
    rclpy.init()
    node = WaypointMarkerPublisher()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
