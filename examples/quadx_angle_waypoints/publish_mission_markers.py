#!/usr/bin/env python3
"""Publish a ROScopter mission file as persistent RViz waypoint markers."""

import argparse
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


def parse_mission(path):
    waypoints = []
    current = []
    for line in Path(path).read_text().splitlines():
        if line.startswith("wp:") and current:
            waypoints.append(yaml.safe_load("\n".join(current))["wp"])
            current = []
        if line.strip() and not line.lstrip().startswith("#"):
            current.append(line)
    if current:
        waypoints.append(yaml.safe_load("\n".join(current))["wp"])
    return waypoints


def point_from_waypoint(waypoint):
    point = Point()
    point.x = float(waypoint["w"][0])
    point.y = float(waypoint["w"][1])
    point.z = float(waypoint["w"][2])
    return point


class MissionMarkerPublisher(Node):
    def __init__(self, mission):
        super().__init__("quadx_angle_mission_marker_publisher")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(Marker, "/rviz/waypoint", qos)
        self.waypoints = parse_mission(mission)
        if not self.waypoints:
            raise RuntimeError(f"No waypoints found in {mission}")
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f"Publishing {len(self.waypoints)} mission waypoints to /rviz/waypoint")

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        points = [point_from_waypoint(waypoint) for waypoint in self.waypoints]

        line = self.base_marker(stamp, "quadx_mission_path", 0, Marker.LINE_STRIP)
        line.scale.x = 0.15
        line.color.r = 0.0
        line.color.g = 0.8
        line.color.b = 0.2
        line.color.a = 1.0
        line.points = points
        self.publisher.publish(line)

        for index, point in enumerate(points, start=1):
            sphere = self.base_marker(stamp, "quadx_mission_points", index, Marker.SPHERE)
            sphere.pose.position = point
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 1.0
            sphere.scale.y = 1.0
            sphere.scale.z = 1.0
            sphere.color.r = 1.0
            sphere.color.g = 0.1
            sphere.color.b = 0.1
            sphere.color.a = 1.0
            self.publisher.publish(sphere)

            label = self.base_marker(stamp, "quadx_mission_labels", index, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = point.x
            label.pose.position.y = point.y
            label.pose.position.z = point.z + 1.5
            label.pose.orientation.w = 1.0
            label.scale.z = 1.2
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"WP{index}"
            self.publisher.publish(label)

    @staticmethod
    def base_marker(stamp, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "NED"
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        return marker


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mission", help="ROScopter mission YAML file")
    args = parser.parse_args()

    rclpy.init()
    node = MissionMarkerPublisher(args.mission)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
