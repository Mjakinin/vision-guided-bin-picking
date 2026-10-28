#!/usr/bin/env python3
"""
Thread-Safe MoveIt 2 Python Interface for Franka Emika Panda.
Utilizes ROS 2 action/service callbacks with thread events to allow clean,
non-blocking execution from background worker threads while rclpy spins in the main thread.
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np
import tf2_ros
import tf_transformations

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotTrajectory,
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    ObjectColor,
)
from std_msgs.msg import ColorRGBA
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState


class MoveIt2Interface:
    def __init__(self, node: Node, group_name: str = "panda_arm", ee_link: str = "panda_link8"):
        self.node = node
        self.group_name = group_name
        self.ee_link = ee_link

        # Action Clients
        self.move_group_client = ActionClient(self.node, MoveGroup, "/move_action")
        self.execute_traj_client = ActionClient(self.node, ExecuteTrajectory, "/execute_trajectory")
        self.arm_client = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.gripper_client = ActionClient(self.node, GripperCommand, "/panda_hand_controller/gripper_cmd")

        # Service Clients
        self.cartesian_client = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.scene_client = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")

        # Attached Collision Object Publisher (for robust attach/detach)
        self.attached_object_pub = self.node.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10
        )

        # TF Buffer for looking up current end-effector pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Joint State tracking
        self.latest_joint_state = None
        self.joint_sub = self.node.create_subscription(
            JointState, "/joint_states", self._joint_callback, 10
        )

    def _joint_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def wait_for_servers(self, timeout_sec: float = 25.0) -> bool:
        """Wait for MoveIt 2 action servers, controllers, and services to become fully active."""
        self.node.get_logger().info("Connecting to MoveIt 2 actions, controllers, and services...")
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            if self.move_group_client.wait_for_server(timeout_sec=0.5) and \
               self.cartesian_client.wait_for_service(timeout_sec=0.5) and \
               self.scene_client.wait_for_service(timeout_sec=0.5) and \
               self.arm_client.wait_for_server(timeout_sec=0.5) and \
               self.gripper_client.wait_for_server(timeout_sec=0.5):
                self.node.get_logger().info("✓ Connected to MoveIt 2 servers and controllers!")
                return True
            time.sleep(0.5)
        self.node.get_logger().warn("Timeout connecting to MoveIt 2 servers.")
        return False

    def get_current_pose(self, base_frame: str = "world") -> PoseStamped:
        """Looks up the current TF pose of the end-effector link."""
        try:
            t = self.tf_buffer.lookup_transform(
                base_frame,
                self.ee_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            p = PoseStamped()
            p.header = t.header
            p.pose.position.x = t.transform.translation.x
            p.pose.position.y = t.transform.translation.y
            p.pose.position.z = t.transform.translation.z
            p.pose.orientation = t.transform.rotation
            return p
        except Exception as e:
            self.node.get_logger().warn(f"Failed to lookup EE transform: {e}")
            return None

    def _call_service_sync(self, client, request, timeout_sec: float = 10.0):
        """Thread-safe synchronous service caller using threading.Event."""
        if not client.service_is_ready():
            return None
        event = threading.Event()
        res_container = [None]

        def _on_done(future):
            try:
                res_container[0] = future.result()
            except Exception as e:
                self.node.get_logger().warn(f"Service call exception: {e}")
            finally:
                event.set()

        future = client.call_async(request)
        future.add_done_callback(_on_done)
        event.wait(timeout=timeout_sec)
        return res_container[0]

    def _call_action_sync(self, client, goal_msg, timeout_sec: float = 25.0):
        """Thread-safe synchronous action caller using threading.Event."""
        if not client.wait_for_server(timeout_sec=2.0):
            return None
        event = threading.Event()
        res_container = [None]

        def _on_goal_sent(goal_future):
            try:
                handle = goal_future.result()
                if not handle.accepted:
                    event.set()
                    return

                def _on_result(res_future):
                    try:
                        res_container[0] = res_future.result()
                    finally:
                        event.set()

                res_f = handle.get_result_async()
                res_f.add_done_callback(_on_result)
            except Exception as e:
                self.node.get_logger().warn(f"Action send exception: {e}")
                event.set()

        send_f = client.send_goal_async(goal_msg)
        send_f.add_done_callback(_on_goal_sent)
        event.wait(timeout=timeout_sec)
        return res_container[0]

    def plan_and_execute_to_pose(self, target_pose: PoseStamped, tolerance: float = 0.03) -> bool:
        """Plans a collision-free OMPL trajectory (RRTConnect) to target pose and executes it."""
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        now_msg = self.node.get_clock().now().to_msg()
        req.workspace_parameters.header.frame_id = "world"
        req.workspace_parameters.header.stamp = now_msg
        req.group_name = self.group_name
        req.planner_id = "RRTConnectkConfigDefault"
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.6
        req.max_acceleration_scaling_factor = 0.6

        # Position Constraint
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "world"
        pos_con.header.stamp = now_msg
        pos_con.link_name = self.ee_link
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [tolerance]
        bv.primitives.append(sp)
        bv.primitive_poses.append(target_pose.pose)
        pos_con.constraint_region = bv
        pos_con.weight = 1.0

        # Orientation Constraint: Keep tool pointing strictly downwards without random spinning
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = "world"
        ori_con.header.stamp = now_msg
        ori_con.link_name = self.ee_link
        ori_con.orientation = target_pose.pose.orientation
        ori_con.absolute_x_axis_tolerance = 0.25
        ori_con.absolute_y_axis_tolerance = 0.25
        ori_con.absolute_z_axis_tolerance = 0.50
        ori_con.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        req.goal_constraints.append(constraints)

        goal_msg.request = req
        goal_msg.planning_options.plan_only = False

        res = self._call_action_sync(self.move_group_client, goal_msg, timeout_sec=20.0)
        return res is not None and (res.result.error_code.val == 1 or res.status == 4)

    def plan_and_execute_cartesian_path(self, waypoints: list, max_step: float = 0.005, avoid_collisions: bool = False) -> bool:
        """
        Computes a strictly linear Cartesian path and executes it.
        Crucial for bin-picking descend/retract to avoid colliding with bin walls.
        By default, avoid_collisions is False during descend/retract to allow contact with target parts.
        """
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = self.group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions

        res = self._call_service_sync(self.cartesian_client, req, timeout_sec=5.0)
        if res is None:
            return False

        fraction = res.fraction
        self.node.get_logger().info(f"Cartesian trajectory fraction completed: {fraction * 100:.1f}%")
        if fraction < 0.60:
            return False

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.solution

        exec_res = self._call_action_sync(self.execute_traj_client, exec_goal, timeout_sec=15.0)
        return exec_res is not None and (exec_res.result.error_code.val == 1 or exec_res.status == 4)

    def control_gripper(self, position: float, max_effort: float = 25.0) -> bool:
        """
        Commands the parallel gripper fingers.
        position: 0.035 for full open (7cm), 0.005 for grasp.
        """
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        res = self._call_action_sync(self.gripper_client, goal, timeout_sec=5.0)
        return res is not None

    def attach_object(self, object_id: str, link_name: str = "panda_hand",
                      color=(0.95, 0.20, 0.20, 1.0)) -> bool:
        """
        Atomically removes workpiece from world bin and attaches it
        cleanly between the parallel gripper finger pads of panda_hand via /apply_planning_scene.
        """
        # Step 1: Remove from world bin
        req1 = ApplyPlanningScene.Request()
        req1.scene.is_diff = True
        co_remove = CollisionObject()
        co_remove.header.frame_id = "world"
        co_remove.header.stamp = self.node.get_clock().now().to_msg()
        co_remove.id = object_id
        co_remove.operation = CollisionObject.REMOVE
        req1.scene.world.collision_objects.append(co_remove)
        self._call_service_sync(self.scene_client, req1, timeout_sec=2.0)

        # Step 2: Attach to panda_hand
        req2 = ApplyPlanningScene.Request()
        req2.scene.is_diff = True
        req2.scene.robot_state.is_diff = True
        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.object.id = object_id
        aco.object.header.frame_id = link_name
        aco.object.header.stamp = self.node.get_clock().now().to_msg()
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_link7", "panda_link8"]

        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [0.05, 0.025]  # 5cm height, 2.5cm radius
        aco.object.primitives.append(cyl)

        # Exact center between finger pads of panda_hand: z = 0.1034m
        p = Pose()
        p.position.x = 0.0
        p.position.y = 0.0
        p.position.z = 0.1034
        p.orientation.w = 1.0
        aco.object.primitive_poses.append(p)
        req2.scene.robot_state.attached_collision_objects.append(aco)

        if color:
            oc = ObjectColor()
            oc.id = object_id
            oc.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]) if len(color) > 3 else 1.0)
            req2.scene.object_colors.append(oc)

        res2 = self._call_service_sync(self.scene_client, req2, timeout_sec=2.0)
        success = res2 is not None and res2.success
        self.node.get_logger().info(f"Attached '{object_id}' to {link_name} via PlanningScene (success={success}).")
        return success

    def detach_object(self, object_id: str, link_name: str = "panda_hand",
                      drop_x: float = 0.48, drop_y: float = -0.22, drop_z: float = 0.06,
                      color=(0.95, 0.20, 0.20, 1.0)) -> bool:
        """
        Atomically detaches workpiece from panda_hand, removes the detached copy MoveIt drops
        at the gripper TCP, and places the workpiece upright at the designated pallet fixture slot.
        """
        # Step 1: Detach from gripper (MoveIt removes it from attached bodies)
        req1 = ApplyPlanningScene.Request()
        req1.scene.is_diff = True
        req1.scene.robot_state.is_diff = True
        aco_remove = AttachedCollisionObject()
        aco_remove.link_name = link_name
        aco_remove.object.id = object_id
        aco_remove.object.header.frame_id = link_name
        aco_remove.object.header.stamp = self.node.get_clock().now().to_msg()
        aco_remove.object.operation = CollisionObject.REMOVE
        req1.scene.robot_state.attached_collision_objects.append(aco_remove)
        self._call_service_sync(self.scene_client, req1, timeout_sec=2.0)

        # Step 2: Remove the world copy that MoveIt automatically creates at current TCP upon detachment
        req2 = ApplyPlanningScene.Request()
        req2.scene.is_diff = True
        co_rem = CollisionObject()
        co_rem.header.frame_id = "world"
        co_rem.header.stamp = self.node.get_clock().now().to_msg()
        co_rem.id = object_id
        co_rem.operation = CollisionObject.REMOVE
        req2.scene.world.collision_objects.append(co_rem)
        self._call_service_sync(self.scene_client, req2, timeout_sec=2.0)

        # Step 3: Add workpiece cleanly at designated pallet fixture location
        req3 = ApplyPlanningScene.Request()
        req3.scene.is_diff = True
        co_add = CollisionObject()
        co_add.header.frame_id = "world"
        co_add.header.stamp = self.node.get_clock().now().to_msg()
        co_add.id = object_id
        co_add.operation = CollisionObject.ADD

        cyl = SolidPrimitive()
        cyl.type = SolidPrimitive.CYLINDER
        cyl.dimensions = [0.05, 0.025]  # 5cm height, 2.5cm radius
        co_add.primitives.append(cyl)

        # Placed upright on pallet (pallet top surface is z = 0.06m, cylinder center is z = 0.06 + 0.025 = 0.085m)
        p = Pose()
        p.position.x = float(drop_x)
        p.position.y = float(drop_y)
        p.position.z = float(drop_z + 0.025)
        p.orientation.w = 1.0
        co_add.primitive_poses.append(p)
        req3.scene.world.collision_objects.append(co_add)

        if color:
            oc = ObjectColor()
            oc.id = object_id
            oc.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]) if len(color) > 3 else 1.0)
            req3.scene.object_colors.append(oc)

        res3 = self._call_service_sync(self.scene_client, req3, timeout_sec=2.0)
        success = res3 is not None and res3.success
        self.node.get_logger().info(f"Placed '{object_id}' on pallet at ({drop_x:.3f}, {drop_y:.3f}) (success={success}).")
        return success
