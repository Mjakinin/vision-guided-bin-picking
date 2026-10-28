#!/usr/bin/env bash
# ==============================================================================
# Vision-Guided Industrial Bin-Picking — Master Launcher
# Franka Emika Panda (7-DOF) + MoveIt 2 + 3D Vision
# ==============================================================================

set -e

# Allow Docker container to access host X11 display (for RViz2)
xhost +local:docker > /dev/null 2>&1 || true

# Ensure ros_container is running
if [ -z "$(docker ps -q -f name=ros_container)" ]; then
    echo "➜ Starte ros_container Docker-Container..."
    docker start ros_container > /dev/null 2>&1 || true
    sleep 1
fi

echo "=============================================================================="
echo "    VISION-GUIDED BIN-PICKING & SMART MANIPULATION (MOVEIT 2)"
echo "=============================================================================="
echo " Bitte wähle eine Option:"
echo " [1] Interaktive Live-Demo (Vollautomatisches Bin-Picking + RViz2 GUI)"
echo " [2] Automatisierter Regressionstest & Performance-Benchmark"
echo " [3] Beenden"
echo "=============================================================================="
read -p "Auswahl [1-3] (Standard: 1): " CHOICE
CHOICE="${CHOICE:-1}"

if [ "$CHOICE" == "1" ]; then
    echo "➜ Bereinige alte Prozesse & initialisiere MoveIt 2 Workcell mit RViz2..."
    docker exec ros_container bash -c "pkill -9 -f 'move_group|rviz2|planning_scene|vision_detector|bin_picking|controller_manager' 2>/dev/null || true; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true"
    sleep 0.5
    docker exec -it ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && ros2 launch vision_guided_bin_picking bin_picking.launch.py auto_orchestrate:=true"

elif [ "$CHOICE" == "2" ]; then
    echo "➜ Bereinige alte Prozesse & führe automatisierten Testlauf aus..."
    docker exec ros_container bash -c "pkill -9 -f 'move_group|rviz2|planning_scene|vision_detector|bin_picking|controller_manager' 2>/dev/null || true; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true"
    sleep 0.5
    docker exec -it ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && python3 /root/ros-projects/src/vision_guided_bin_picking/tools/test_bin_picking.py"

else
    echo "Beendet."
    exit 0
fi
