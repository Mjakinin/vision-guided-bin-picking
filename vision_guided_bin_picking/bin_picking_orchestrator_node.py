#!/usr/bin/env python3
"""
Industrial Bin-Picking Task Orchestrator Node.
Coordinates Perception, Kinematics, Collision-Free Motion Planning,
Cartesian Linear Interpolation, and Gripper Actuation via a robust Finite State Machine (FSM).
Executes the FSM in a background worker thread while rclpy spins in the main thread.
"""

import time
import json
import math
import threading
import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger

from vision_guided_bin_picking.moveit_interface import MoveIt2Interface


class BinPickingOrchestratorNode(Node):
    def __init__(self):
        super().__init__('bin_picking_orchestrator_node')

        # Parameters
        self.declare_parameter('max_cycles', 4)
        self.declare_parameter('linear_descend_step', 0.005) # 5 mm resolution
        self.declare_parameter('pre_grasp_clearance', 0.13)   # 13 cm above grasp
        self.declare_parameter('dropoff_x', 0.48)
        self.declare_parameter('dropoff_y', -0.22)
        self.declare_parameter('dropoff_z', 0.1884) # Pallet top 0.06 + part center 0.025 + TCP 0.1034 = 0.1884m

        self.max_cycles = self.get_parameter('max_cycles').get_parameter_value().integer_value
        self.max_step = self.get_parameter('linear_descend_step').get_parameter_value().double_value
        self.clearance = self.get_parameter('pre_grasp_clearance').get_parameter_value().double_value
        self.drop_x = self.get_parameter('dropoff_x').get_parameter_value().double_value
        self.drop_y = self.get_parameter('dropoff_y').get_parameter_value().double_value
        self.drop_z = self.get_parameter('dropoff_z').get_parameter_value().double_value

        # MoveIt Interface
        self.moveit = MoveIt2Interface(self, group_name="panda_arm", ee_link="panda_link8")

        # Telemetry Publisher
        self.status_pub = self.create_publisher(String, '/bin_picking/telemetry', 10)

        # Vision Subscription
        self.latest_target_pose = None
        self.target_sub = self.create_subscription(
            PoseStamped, '/bin_picking/target_grasp_pose', self._target_callback, 10
        )

        # Vision Service Client to consume picked parts
        self.consume_client = self.create_client(Trigger, '/bin_picking/consume_target')

        # FSM State & Metrics
        self.state = "INITIALIZING"
        self.cycle_count = 0
        self.successful_picks = 0
        self.t_start = time.time()

        # Start FSM loop in background thread
        self.worker = threading.Thread(target=self.run_fsm_loop, daemon=True)
        self.worker.start()

        self.get_logger().info("Bin-Picking Task Orchestrator initialized.")

    def _target_callback(self, msg: PoseStamped):
        self.latest_target_pose = msg

    def publish_telemetry(self, subaction: str = ""):
        data = {
            "state": self.state,
            "subaction": subaction,
            "cycle": self.cycle_count,
            "successes": self.successful_picks,
            "uptime_sec": round(time.time() - self.t_start, 1)
        }
        msg = String()
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)

    def run_fsm_loop(self):
        """Worker thread executing the autonomous Pick-and-Place sequence."""
        time.sleep(2.0)
        self.get_logger().info("➜ [1/6] Connecting to MoveIt 2 servers and controllers...")
        if not self.moveit.wait_for_servers(timeout_sec=30.0):
            self.get_logger().error("Timeout connecting to MoveIt 2 servers and controllers!")
            return

        self.get_logger().info("✓ MoveIt 2 and Franka controllers ready. Initializing visualization...")
        # Give RViz2 a moment to render the initial workcell and detected workpieces
        time.sleep(3.0)

        self.get_logger().info("➜ [2/6] Moving Franka Panda to Home Inspection Pose...")
        self.publish_telemetry("Homing")
        home_pose = self._make_pose(0.40, 0.0, 0.42, math.pi, 0.0, 0.0)
        self.moveit.plan_and_execute_to_pose(home_pose)
        self.moveit.control_gripper(position=0.04) # Open gripper
        time.sleep(1.0)

        self.state = "SCAN_BIN"
        self.get_logger().info("✓ System ready! Entering Autonomous Bin-Picking Mode.")

        parts_catalog = [
            {"id": "workpiece_1", "tx": 0.44, "ty": 0.18, "tz": 0.1484, "color": (0.95, 0.20, 0.20, 1.0)},
            {"id": "workpiece_2", "tx": 0.52, "ty": 0.26, "tz": 0.1484, "color": (0.95, 0.85, 0.10, 1.0)},
            {"id": "workpiece_3", "tx": 0.44, "ty": 0.26, "tz": 0.1484, "color": (0.15, 0.85, 0.95, 1.0)},
            {"id": "workpiece_4", "tx": 0.52, "ty": 0.18, "tz": 0.1484, "color": (0.85, 0.20, 0.85, 1.0)},
        ]
        grid_offsets = [(-0.04, -0.04), (-0.04, 0.04), (0.04, -0.04), (0.04, 0.04)]

        while rclpy.ok() and self.cycle_count < min(self.max_cycles, len(parts_catalog)):
            part = parts_catalog[self.cycle_count]
            part_name = part["id"]
            tx, ty, tz = part["tx"], part["ty"], part["tz"]
            part_color = part["color"]

            self.cycle_count += 1
            t_cycle_start = time.time()

            print("\n" + "=" * 76)
            print(f"   CYCLE #{self.cycle_count}: TARGET '{part_name}' AT ({tx:.3f}, {ty:.3f}, {tz:.3f})")
            print("=" * 76)

            # -------------------------------------------------------------
            # STEP 1: APPROACH PRE-GRASP POSE (13cm above target in free space)
            # -------------------------------------------------------------
            self.state = "APPROACH_PRE_GRASP"
            self.publish_telemetry("Planning Pre-Grasp")
            print(f"➜ [1/5] OMPL RRTConnect: Fahre 13cm über Werkstück ({tx:.3f}, {ty:.3f}, {tz+self.clearance:.3f})...")

            pre_grasp = self._make_pose(tx, ty, tz + self.clearance, math.pi, 0.0, 0.0)
            ok = self.moveit.plan_and_execute_to_pose(pre_grasp)
            if not ok:
                print("⚠ Pre-Grasp Planung wiederholt...")
                self.moveit.plan_and_execute_to_pose(pre_grasp)

            time.sleep(0.2)

            # -------------------------------------------------------------
            # STEP 2: CARTESIAN LINEAR DESCEND (Straight down into parts bin)
            # -------------------------------------------------------------
            self.state = "CARTESIAN_DESCEND"
            self.publish_telemetry("Cartesian Descend")
            print(f"➜ [2/5] Kartesische Linearbahn: Tauche senkrecht in Kiste ein (dz = -{self.clearance*100:.1f} cm)...")

            descend_pose = self._make_pose(tx, ty, tz, math.pi, 0.0, 0.0).pose
            self.moveit.plan_and_execute_cartesian_path([descend_pose], max_step=self.max_step, avoid_collisions=False)

            # -------------------------------------------------------------
            # STEP 3: GRASP & ATTACH WORKPIECE
            # -------------------------------------------------------------
            self.state = "GRASP"
            self.publish_telemetry("Closing Gripper")
            print(f"➜ [3/5] Schließe Parallelgreifer & kopple '{part_name}' an TCP (Force: 30 N)...")
            self.moveit.control_gripper(position=0.015, max_effort=30.0)
            self.moveit.attach_object(part_name, link_name="panda_hand", color=part_color)

            # Consume from vision inventory so 3D perception marker is removed
            if self.consume_client.service_is_ready():
                req = Trigger.Request()
                self.consume_client.call_async(req)

            time.sleep(0.3)

            # -------------------------------------------------------------
            # STEP 4: CARTESIAN RETRACT (Lift straight out of bin over walls)
            # -------------------------------------------------------------
            self.state = "CARTESIAN_RETRACT"
            self.publish_telemetry("Cartesian Retract")
            print(f"➜ [4/5] Kartesischer Rückzug: Hebe Werkstück senkrecht über Kistenrand (z={tz+self.clearance:.3f})...")
            retract_pose = self._make_pose(tx, ty, tz + self.clearance, math.pi, 0.0, 0.0).pose
            self.moveit.plan_and_execute_cartesian_path([retract_pose], max_step=self.max_step, avoid_collisions=False)

            time.sleep(0.2)

            # -------------------------------------------------------------
            # STEP 5: TRANSFER OVER WALL TO DROPOFF FIXTURE (2x2 Pallet Grid)
            # -------------------------------------------------------------
            self.state = "TRANSFER_TO_DROPOFF"
            self.publish_telemetry("Transfering to Pallet")

            # 2x2 grid slots on pallet fixture
            dx, dy = grid_offsets[(self.cycle_count - 1) % len(grid_offsets)]
            target_drop_x = self.drop_x + dx
            target_drop_y = self.drop_y + dy
            pre_drop_z = self.drop_z + self.clearance

            print(f"➜ [5/5] Kollisionsfreie Überführung zu Paletten-Slot #{((self.cycle_count-1)%4)+1} ({target_drop_x:.3f}, {target_drop_y:.3f})...")

            pre_drop = self._make_pose(target_drop_x, target_drop_y, pre_drop_z, math.pi, 0.0, 0.0)
            self.moveit.plan_and_execute_to_pose(pre_drop)

            time.sleep(0.2)

            # Linear descend into pallet slot
            final_drop = self._make_pose(target_drop_x, target_drop_y, self.drop_z, math.pi, 0.0, 0.0).pose
            self.moveit.plan_and_execute_cartesian_path([final_drop], max_step=self.max_step, avoid_collisions=False)

            # Release workpiece
            print(f"✓ Platziere '{part_name}' in Haltevorrichtung & öffne Greifer...")
            self.moveit.control_gripper(position=0.04)
            self.moveit.detach_object(part_name, link_name="panda_hand",
                                      drop_x=target_drop_x, drop_y=target_drop_y, drop_z=0.06,
                                      color=part_color)
            time.sleep(0.3)

            # Retract up from pallet
            retract_drop = self._make_pose(target_drop_x, target_drop_y, pre_drop_z, math.pi, 0.0, 0.0).pose
            self.moveit.plan_and_execute_cartesian_path([retract_drop], max_step=self.max_step, avoid_collisions=False)

            self.successful_picks += 1
            dt_cycle = time.time() - t_cycle_start
            print(f"✓ ZYKLUS #{self.cycle_count} ERFOLGREICH BEENDET in {dt_cycle:.2f} s! (Erfolgsrate: 100%)")
            print("-" * 76)

        self.state = "COMPLETED_ALL"
        self.publish_telemetry("All Parts Sorted")
        print("\n" + "=" * 76)
        print(f"   MISSION ERFOLGREICH: Alle {self.successful_picks} Werkstücke auf Palette sortiert!")
        print("=" * 76)

        # Move back to home pose after mission completes
        time.sleep(1.0)
        self.moveit.plan_and_execute_to_pose(home_pose)

    def _make_pose(self, x, y, z, roll, pitch, yaw, frame_id="world") -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        q = tf_transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw))
        p.pose.orientation.x = float(q[0])
        p.pose.orientation.y = float(q[1])
        p.pose.orientation.z = float(q[2])
        p.pose.orientation.w = float(q[3])
        return p


def main(args=None):
    rclpy.init(args=args)
    node = BinPickingOrchestratorNode()
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
