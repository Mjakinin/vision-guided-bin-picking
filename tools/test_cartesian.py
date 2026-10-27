#!/usr/bin/env python3
import rclpy
import math
import time
from geometry_msgs.msg import Pose, PoseStamped
import tf_transformations
from vision_guided_bin_picking.moveit_interface import MoveIt2Interface

def test():
    rclpy.init()
    node = rclpy.create_node('test_cartesian_client')
    moveit = MoveIt2Interface(node, group_name="panda_arm", ee_link="panda_link8")
    
    import threading
    worker = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    worker.start()

    print("Connecting to MoveIt...")
    if not moveit.wait_for_servers(timeout_sec=5.0):
        print("MoveIt not ready!")
        return

    # 1. Plan to pre-grasp
    tx, ty, tz = 0.43, 0.21, 0.175
    clearance = 0.12
    q = tf_transformations.quaternion_from_euler(math.pi, 0.0, 0.0)

    pre_grasp = PoseStamped()
    pre_grasp.header.frame_id = "world"
    pre_grasp.pose.position.x = tx
    pre_grasp.pose.position.y = ty
    pre_grasp.pose.position.z = tz + clearance
    pre_grasp.pose.orientation.x = float(q[0])
    pre_grasp.pose.orientation.y = float(q[1])
    pre_grasp.pose.orientation.z = float(q[2])
    pre_grasp.pose.orientation.w = float(q[3])

    print("1. Planning to Pre-Grasp:", (tx, ty, tz + clearance))
    ok1 = moveit.plan_and_execute_to_pose(pre_grasp)
    print("Pre-grasp execution result:", ok1)

    time.sleep(1.0)

    # 2. Cartesian descend
    target_pose = Pose()
    target_pose.position.x = tx
    target_pose.position.y = ty
    target_pose.position.z = tz
    target_pose.orientation = pre_grasp.pose.orientation

    print("2. Planning Cartesian Descend to:", (tx, ty, tz))
    from moveit_msgs.srv import GetCartesianPath
    req = GetCartesianPath.Request()
    req.header.frame_id = "world"
    req.group_name = "panda_arm"
    req.link_name = "panda_link8"
    req.waypoints = [pre_grasp.pose, target_pose]
    req.max_step = 0.005
    req.jump_threshold = 0.0
    req.avoid_collisions = True

    res = moveit._call_service_sync(moveit.cartesian_client, req)
    if res:
        print(f"Cartesian Fraction: {res.fraction*100:.1f}%, Error code: {res.error_code.val}")
        if res.fraction > 0.8:
            print("Executing trajectory...")
            exec_goal = moveit.execute_traj_client._action_type.Goal()
            exec_goal.trajectory = res.solution
            exec_res = moveit._call_action_sync(moveit.execute_traj_client, exec_goal)
            print("Execution result:", exec_res.status if exec_res else "None")
    else:
        print("Cartesian service returned None!")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    test()
