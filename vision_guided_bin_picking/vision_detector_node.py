#!/usr/bin/env python3
"""
3D Vision Detector Node for Industrial Bin-Picking.
Simulates high-precision 3D RGB-D camera / Point Cloud perception:
- Identifies workpieces inside the parts bin.
- Computes 6D Grasp Poses (Position + Downward Orientation + Optimal Grasp Yaw).
- Evaluates Grasp Quality Metric based on wall clearance.
- Publishes RViz 3D markers and provides target pose to the orchestrator.
"""

import math
import random
import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger


class VisionDetectorNode(Node):
    def __init__(self):
        super().__init__('vision_detector_node')

        # Parameters
        self.declare_parameter('bin_center_x', 0.48)
        self.declare_parameter('bin_center_y', 0.25)
        self.declare_parameter('bin_floor_z', 0.05)
        self.declare_parameter('part_height', 0.04)

        self.bin_x = self.get_parameter('bin_center_x').get_parameter_value().double_value
        self.bin_y = self.get_parameter('bin_center_y').get_parameter_value().double_value
        self.bin_z = self.get_parameter('bin_floor_z').get_parameter_value().double_value
        self.part_h = self.get_parameter('part_height').get_parameter_value().double_value

        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, '/bin_picking/target_grasp_pose', 10)
        self.parts_pub = self.create_publisher(PoseArray, '/bin_picking/detected_parts', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/bin_picking/part_markers', 10)

        # Service to query next best workpiece for picking
        self.srv = self.create_service(Trigger, '/bin_picking/get_next_target', self.get_next_target_callback)
        self.consume_srv = self.create_service(Trigger, '/bin_picking/consume_target', self.consume_target_callback)

        # Workpiece inventory in the bin (x_offset, y_offset, yaw_deg, color)
        # Offsets relative to bin center (0.48, 0.25)
        self.initial_parts = [
            {"id": "part_alpha", "dx": -0.05, "dy": -0.04, "yaw": 0.0, "color": (0.9, 0.2, 0.2)},   # Red
            {"id": "part_beta",  "dx":  0.06, "dy":  0.05, "yaw": 45.0, "color": (0.9, 0.8, 0.1)},  # Yellow
            {"id": "part_gamma", "dx": -0.02, "dy":  0.06, "yaw": -30.0, "color": (0.2, 0.8, 0.9)}, # Cyan
            {"id": "part_delta", "dx":  0.04, "dy": -0.06, "yaw": 60.0, "color": (0.8, 0.2, 0.8)},  # Magenta
        ]
        self.active_parts = list(self.initial_parts)

        # Periodic detection and marker publish loop (2 Hz)
        self.timer = self.create_timer(0.5, self.detection_loop)
        self.get_logger().info(f"3D Vision Detector initialized with {len(self.active_parts)} workpieces in bin.")

    def compute_grasp_pose(self, part: dict) -> PoseStamped:
        """
        Computes 6D grasp pose for Franka Panda.
        Gripper points straight down (-Z in world):
        Euler angles: roll=pi, pitch=0.0, yaw=part_yaw
        """
        px = self.bin_x + part["dx"]
        py = self.bin_y + part["dy"]
        # Grasp Z for panda_link8 (accounting for finger length 10.5cm from link8 to tips)
        pz = self.bin_z + self.part_h + 0.085

        # Quaternions for pointing down with custom yaw
        # In Franka tool frame, z-axis points out of gripper.
        # Rotating by 180 deg around X (or roll=pi, pitch=0) points tool down.
        yaw_rad = math.radians(part["yaw"])
        q = tf_transformations.quaternion_from_euler(math.pi, 0.0, yaw_rad)

        target = PoseStamped()
        target.header.frame_id = "world"
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = float(px)
        target.pose.position.y = float(py)
        target.pose.position.z = float(pz)
        target.pose.orientation.x = float(q[0])
        target.pose.orientation.y = float(q[1])
        target.pose.orientation.z = float(q[2])
        target.pose.orientation.w = float(q[3])
        return target

    def get_best_part(self):
        """
        Calculates grasp quality based on distance from bin boundary walls.
        Parts closest to the bin center have the highest clearance.
        """
        if not self.active_parts:
            return None
        # Sort by lowest radial offset from bin center (highest wall clearance)
        sorted_parts = sorted(self.active_parts, key=lambda p: math.hypot(p["dx"], p["dy"]))
        return sorted_parts[0]

    def detection_loop(self):
        """Publishes detected parts and visual markers."""
        best = self.get_best_part()
        if best:
            target_pose = self.compute_grasp_pose(best)
            self.target_pub.publish(target_pose)

        # Publish PoseArray
        pa = PoseArray()
        pa.header.frame_id = "world"
        pa.header.stamp = self.get_clock().now().to_msg()
        for p in self.active_parts:
            target = self.compute_grasp_pose(p)
            pa.poses.append(target.pose)
        self.parts_pub.publish(pa)

        # Publish MarkerArray
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, p in enumerate(self.active_parts):
            px = self.bin_x + p["dx"]
            py = self.bin_y + p["dy"]
            pz = self.bin_z + self.part_h / 2.0

            # Workpiece body (Cylinder / Industrial Bushing)
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = now
            m.ns = "parts"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(px)
            m.pose.position.y = float(py)
            m.pose.position.z = float(pz)

            yaw_rad = math.radians(p["yaw"])
            q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)
            m.pose.orientation.x = float(q[0])
            m.pose.orientation.y = float(q[1])
            m.pose.orientation.z = float(q[2])
            m.pose.orientation.w = float(q[3])

            m.scale.x = 0.05  # Diameter
            m.scale.y = 0.05
            m.scale.z = float(self.part_h)

            m.color.r = float(p["color"][0])
            m.color.g = float(p["color"][1])
            m.color.b = float(p["color"][2])
            m.color.a = 0.95
            ma.markers.append(m)

            # Grasp Arrow Marker
            arrow = Marker()
            arrow.header.frame_id = "world"
            arrow.header.stamp = now
            arrow.ns = "grasp_vectors"
            arrow.id = 100 + i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            # Start point: 6cm above part
            p_start = Point(x=float(px), y=float(py), z=float(pz + 0.08))
            # End point: at part top
            p_end = Point(x=float(px), y=float(py), z=float(pz + 0.01))
            arrow.points = [p_start, p_end]

            arrow.scale.x = 0.008  # Shaft diameter
            arrow.scale.y = 0.015  # Head diameter
            arrow.scale.z = 0.02   # Head length

            # Highlight best candidate in bright green, others white
            if p == best:
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (0.1, 1.0, 0.2, 1.0)
            else:
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (0.8, 0.8, 0.8, 0.5)
            ma.markers.append(arrow)

        self.marker_pub.publish(ma)

    def get_next_target_callback(self, request, response):
        best = self.get_best_part()
        if best:
            response.success = True
            response.message = f"Selected target {best['id']} at ({self.bin_x+best['dx']:.3f}, {self.bin_y+best['dy']:.3f})"
        else:
            response.success = False
            response.message = "No remaining workpieces in bin."
        return response

    def consume_target_callback(self, request, response):
        """Called when a part is successfully grasped and moved to dropoff."""
        best = self.get_best_part()
        if best:
            part_id = best['id']
            self.active_parts.remove(best)
            response.success = True
            response.message = f"Consumed {part_id}. Remaining: {len(self.active_parts)}"
            self.get_logger().info(f"✓ Workpiece {part_id} picked! {len(self.active_parts)} parts left in bin.")
            self.detection_loop()
        else:
            response.success = False
            response.message = "No parts to consume."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
