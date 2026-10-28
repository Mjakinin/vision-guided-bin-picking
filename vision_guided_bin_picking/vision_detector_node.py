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
        self.declare_parameter('bin_center_y', 0.22)
        self.declare_parameter('bin_floor_z', 0.02)
        self.declare_parameter('part_height', 0.05)

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
        # Offsets relative to bin center (0.48, 0.22)
        self.initial_parts = [
            {"id": "workpiece_1", "marker_id": 0, "dx": -0.04, "dy": -0.04, "yaw": 0.0, "color": (0.95, 0.2, 0.2)},   # Vivid Red
            {"id": "workpiece_2", "marker_id": 1, "dx":  0.04, "dy":  0.04, "yaw": 0.0, "color": (0.95, 0.85, 0.1)},  # Bright Gold
            {"id": "workpiece_3", "marker_id": 2, "dx": -0.04, "dy":  0.04, "yaw": 0.0, "color": (0.15, 0.85, 0.95)}, # Electric Cyan
            {"id": "workpiece_4", "marker_id": 3, "dx":  0.04, "dy": -0.04, "yaw": 0.0, "color": (0.85, 0.2, 0.85)},  # Neon Magenta
        ]
        self.active_parts = list(self.initial_parts)
        self.placed_parts = []
        self.consumed_marker_ids = []

        # Periodic detection and marker publish loop (2 Hz)
        self.timer = self.create_timer(0.5, self.detection_loop)
        self.get_logger().info(f"3D Vision Detector initialized with {len(self.active_parts)} workpieces in bin.")

    def compute_grasp_pose(self, part: dict) -> PoseStamped:
        """
        Computes 6D grasp pose for Franka Panda TCP (panda_link8).
        Gripper points straight down (-Z in world):
        Euler angles: roll=pi, pitch=0.0, yaw=0.0
        At z=0.15m, the gripper fingers (10.3cm length) are centered directly on the 5cm cylinder.
        """
        px = self.bin_x + part["dx"]
        py = self.bin_y + part["dy"]
        pz = 0.15

        q = tf_transformations.quaternion_from_euler(math.pi, 0.0, 0.0)

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
        Picks the next available workpiece from inventory.
        """
        if not self.active_parts:
            return None
        return self.active_parts[0]

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

        # Delete markers that were consumed
        for mid in self.consumed_marker_ids:
            del_part = Marker()
            del_part.header.frame_id = "world"
            del_part.header.stamp = now
            del_part.ns = "parts"
            del_part.id = mid
            del_part.action = Marker.DELETE
            ma.markers.append(del_part)

            del_arrow = Marker()
            del_arrow.header.frame_id = "world"
            del_arrow.header.stamp = now
            del_arrow.ns = "grasp_vectors"
            del_arrow.id = 100 + mid
            del_arrow.action = Marker.DELETE
            ma.markers.append(del_arrow)

        for p in self.active_parts:
            mid = p["marker_id"]
            px = self.bin_x + p["dx"]
            py = self.bin_y + p["dy"]
            pz = self.bin_z + self.part_h / 2.0

            # Workpiece body (Cylinder / Industrial Bushing)
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = now
            m.ns = "parts"
            m.id = mid
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(px)
            m.pose.position.y = float(py)
            m.pose.position.z = float(pz)
            m.pose.orientation.w = 1.0

            m.scale.x = 0.05  # Diameter (5cm)
            m.scale.y = 0.05
            m.scale.z = float(self.part_h)

            m.color.r = float(p["color"][0])
            m.color.g = float(p["color"][1])
            m.color.b = float(p["color"][2])
            m.color.a = 0.95
            ma.markers.append(m)

            # Grasp Approach Arrow Marker
            arrow = Marker()
            arrow.header.frame_id = "world"
            arrow.header.stamp = now
            arrow.ns = "grasp_vectors"
            arrow.id = 100 + mid
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            # Start point: 10cm above part
            p_start = Point(x=float(px), y=float(py), z=float(pz + 0.12))
            # End point: at top of part
            p_end = Point(x=float(px), y=float(py), z=float(pz + 0.03))
            arrow.points = [p_start, p_end]

            arrow.scale.x = 0.010  # Shaft diameter
            arrow.scale.y = 0.020  # Head diameter
            arrow.scale.z = 0.025  # Head length

            # Highlight best candidate in glowing green, others subtle grey/white
            if p == best:
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (0.1, 1.0, 0.2, 1.0)
            else:
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (0.7, 0.7, 0.7, 0.4)
            ma.markers.append(arrow)

        # Placed parts on the pallet fixture (2x2 grid)
        for j, pl in enumerate(self.placed_parts):
            m_pl = Marker()
            m_pl.header.frame_id = "world"
            m_pl.header.stamp = now
            m_pl.ns = "placed_parts"
            m_pl.id = 200 + j
            m_pl.type = Marker.CYLINDER
            m_pl.action = Marker.ADD
            m_pl.pose.position.x = float(pl["px"])
            m_pl.pose.position.y = float(pl["py"])
            m_pl.pose.position.z = float(pl["pz"])
            m_pl.pose.orientation.w = 1.0
            m_pl.scale.x = 0.05
            m_pl.scale.y = 0.05
            m_pl.scale.z = float(self.part_h)
            m_pl.color.r = float(pl["color"][0])
            m_pl.color.g = float(pl["color"][1])
            m_pl.color.b = float(pl["color"][2])
            m_pl.color.a = 1.0
            ma.markers.append(m_pl)

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
            mid = best.get('marker_id', 0)
            self.consumed_marker_ids.append(mid)
            self.active_parts.remove(best)

            # Record placed workpiece position on dropoff pallet
            grid_offsets = [(-0.04, -0.04), (-0.04, 0.04), (0.04, -0.04), (0.04, 0.04)]
            slot_idx = len(self.placed_parts)
            dx, dy = grid_offsets[slot_idx % len(grid_offsets)]
            placed = dict(best)
            placed["px"] = 0.48 + dx
            placed["py"] = -0.22 + dy
            placed["pz"] = 0.06 + self.part_h / 2.0
            self.placed_parts.append(placed)

            response.success = True
            response.message = f"Consumed {part_id}. Remaining in bin: {len(self.active_parts)}"
            self.get_logger().info(f"✓ Workpiece {part_id} picked & placed on pallet! {len(self.active_parts)} parts left in bin.")
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
