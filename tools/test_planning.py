#!/usr/bin/env python3
import rclpy
import math
import time
from geometry_msgs.msg import PoseStamped
from vision_guided_bin_picking.moveit_interface import MoveIt2Interface

def test():
    rclpy.init()
    node = rclpy.create_node('test_planning_client')
    moveit = MoveIt2Interface(node, group_name="panda_arm", ee_link="panda_link8")
    
    print("Waiting for MoveIt...")
    if not moveit.wait_for_servers(timeout_sec=5.0):
        print("MoveIt not ready!")
        return

    # Test home pose
    p = PoseStamped()
    p.header.frame_id = "world"
    p.pose.position.x = 0.35
    p.pose.position.y = 0.0
    p.pose.position.z = 0.55
    p.pose.orientation.x = 1.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = 0.0
    p.pose.orientation.w = 0.0

    print("Planning to Home Pose...")
    ok = moveit.plan_and_execute_to_pose(p)
    print("Plan result:", ok)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    test()
