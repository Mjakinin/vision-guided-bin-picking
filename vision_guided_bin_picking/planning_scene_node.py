#!/usr/bin/env python3
"""
Planning Scene Node for Industrial Bin-Picking Workcell.
Populates MoveIt 2's collision world with:
- Workstation Table
- Parts Bin with 4 vertical boundary walls (forces vertical entry and exit)
- Drop-off Assembly Pallet / Fixture
- Safety Barrier
Also publishes rich RViz visualization markers.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class PlanningSceneNode(Node):
    def __init__(self):
        super().__init__('planning_scene_node')

        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.marker_pub = self.create_publisher(MarkerArray, '/bin_picking/environment_markers', 10)

        # Periodic republishing of RViz visual markers (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_markers)

        # Initialize scene after MoveIt 2 is up
        self.init_timer = self.create_timer(2.0, self.setup_environment)
        self.environment_initialized = False

        self.get_logger().info("Planning Scene Node started. Waiting for /apply_planning_scene...")

    def setup_environment(self):
        if self.environment_initialized:
            return

        if not self.scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for MoveIt /apply_planning_scene service...")
            return

        self.get_logger().info("➜ Spawning industrial bin-picking workcell collision geometry into MoveIt 2...")
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True

        objects = []

        # 1. Workstation Table Base
        table = self._create_box("workstation_table", 0.45, 0.0, -0.02, 1.2, 1.2, 0.04)
        objects.append(table)

        # 2. Safety Backplate (Industrial safety enclosure rear wall)
        backplate = self._create_box("safety_backplate", -0.45, 0.0, 0.45, 0.05, 1.2, 0.9)
        objects.append(backplate)

        # 3. Parts Bin (Floor + 4 Vertical Walls)
        # Center of Bin: x=0.48, y=0.25
        bin_floor = self._create_box("bin_floor", 0.48, 0.25, 0.02, 0.32, 0.32, 0.02)
        objects.append(bin_floor)

        # North Wall (+y)
        wall_n = self._create_box("bin_wall_north", 0.48, 0.41, 0.09, 0.32, 0.015, 0.14)
        objects.append(wall_n)

        # South Wall (-y)
        wall_s = self._create_box("bin_wall_south", 0.48, 0.09, 0.09, 0.32, 0.015, 0.14)
        objects.append(wall_s)

        # East Wall (+x)
        wall_e = self._create_box("bin_wall_east", 0.64, 0.25, 0.09, 0.015, 0.32, 0.14)
        objects.append(wall_e)

        # West Wall (-x)
        wall_w = self._create_box("bin_wall_west", 0.32, 0.25, 0.09, 0.015, 0.32, 0.14)
        objects.append(wall_w)

        # 4. Drop-off Assembly Fixture / Pallet
        # Center of Drop-off: x=0.48, y=-0.25
        pallet = self._create_box("dropoff_pallet", 0.48, -0.25, 0.04, 0.28, 0.28, 0.08)
        objects.append(pallet)

        req.scene.world.collision_objects = objects

        future = self.scene_client.call_async(req)
        future.add_done_callback(self._on_scene_applied)

    def _on_scene_applied(self, future):
        res = future.result()
        if res and res.success:
            self.environment_initialized = True
            self.get_logger().info("✓ MoveIt 2 Workcell Planning Scene successfully loaded!")
            self.publish_markers()
        else:
            self.get_logger().warn("Retrying workcell collision initialization...")

    def _create_box(self, name, x, y, z, sx, sy, sz, frame_id="world"):
        co = CollisionObject()
        co.header.frame_id = frame_id
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = name
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(sx), float(sy), float(sz)]

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        co.primitives.append(box)
        co.primitive_poses.append(pose)
        return co

    def publish_markers(self):
        """Publishes rich colored markers for RViz visualization."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Table Marker (Grey)
        m_table = self._make_marker(0, "workstation_table", 0.45, 0.0, -0.02, 1.2, 1.2, 0.04, (0.3, 0.3, 0.35, 1.0), now)
        ma.markers.append(m_table)

        # Bin Base (Dark Blue)
        m_bin_base = self._make_marker(1, "bin_floor", 0.48, 0.25, 0.02, 0.32, 0.32, 0.02, (0.1, 0.2, 0.5, 0.8), now)
        ma.markers.append(m_bin_base)

        # Bin Walls (Translucent Blue Glass / Acrylic)
        m_wn = self._make_marker(2, "bin_wn", 0.48, 0.41, 0.09, 0.32, 0.015, 0.14, (0.2, 0.4, 0.8, 0.5), now)
        m_ws = self._make_marker(3, "bin_ws", 0.48, 0.09, 0.09, 0.32, 0.015, 0.14, (0.2, 0.4, 0.8, 0.5), now)
        m_we = self._make_marker(4, "bin_we", 0.64, 0.25, 0.09, 0.015, 0.32, 0.14, (0.2, 0.4, 0.8, 0.5), now)
        m_ww = self._make_marker(5, "bin_ww", 0.32, 0.25, 0.09, 0.015, 0.32, 0.14, (0.2, 0.4, 0.8, 0.5), now)
        ma.markers.extend([m_wn, m_ws, m_we, m_ww])

        # Drop-off Pallet (Industrial Green / CNC Fixture)
        m_pallet = self._make_marker(6, "dropoff_pallet", 0.48, -0.25, 0.04, 0.28, 0.28, 0.08, (0.1, 0.6, 0.3, 0.9), now)
        ma.markers.append(m_pallet)

        # Backplate (Steel Grey)
        m_back = self._make_marker(7, "safety_backplate", -0.45, 0.0, 0.45, 0.05, 1.2, 0.9, (0.4, 0.4, 0.45, 0.6), now)
        ma.markers.append(m_back)

        self.marker_pub.publish(ma)

    def _make_marker(self, marker_id, name, x, y, z, sx, sy, sz, color, stamp):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = stamp
        m.ns = "workcell"
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = float(sx)
        m.scale.y = float(sy)
        m.scale.z = float(sz)
        m.color.r = float(color[0])
        m.color.g = float(color[1])
        m.color.b = float(color[2])
        m.color.a = float(color[3])
        return m


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
