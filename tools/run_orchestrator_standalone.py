#!/usr/bin/env python3
import time
import rclpy
import threading
from vision_guided_bin_picking.bin_picking_orchestrator_node import BinPickingOrchestratorNode

def main():
    rclpy.init()
    node = BinPickingOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
