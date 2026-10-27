#!/usr/bin/env bash
# ==============================================================================
# Vision-Guided Industrial Bin-Picking — Master Launcher
# Franka Emika Panda (7-DOF) + MoveIt 2 + 3D Vision
# ==============================================================================

set -e

# Allow Docker container to access host X11 display (for RViz2)
xhost +local:docker > /dev/null 2>&1 || true

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
    echo "➜ Starte MoveIt 2 Workcell, 3D Vision & Task-Orchestrator mit RViz2..."
    docker exec -it ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && ros2 launch vision_guided_bin_picking bin_picking.launch.py auto_orchestrate:=true"

elif [ "$CHOICE" == "2" ]; then
    echo "➜ Führe automatisierten Testlauf & Trajektorien-Prüfung aus..."
    docker exec -it ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && python3 /root/ros-projects/src/vision_guided_bin_picking/tools/test_bin_picking.py"

else
    echo "Beendet."
    exit 0
fi
