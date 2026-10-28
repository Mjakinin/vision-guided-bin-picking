#!/usr/bin/env bash
# ==============================================================================
# Vision-Guided Industrial Bin-Picking — Master Launcher
# Franka Emika Panda (7-DOF) + MoveIt 2 + 3D Vision
# ==============================================================================

# Allow Docker container to access host X11 display (for RViz2)
xhost +local:docker > /dev/null 2>&1 || true

# Ensure ros_container is running
if [ -z "$(docker ps -q -f name=ros_container)" ]; then
    echo "➜ Starte ros_container Docker-Container..."
    docker start ros_container > /dev/null 2>&1 || true
    sleep 1
fi

cleanup_workcell() {
    echo "➜ Bereinige alte Prozesse & FastDDS Shared Memory..."
    docker exec ros_container python3 -c '
import os, signal, re
my_pid = os.getpid()
parent_pid = os.getppid()
ignore = {1, my_pid, parent_pid}
target = re.compile(r"move_group|rviz2|planning_scene_node|vision_detector_node|bin_picking_orchestrator|controller_manager")
for pid_str in os.listdir("/proc"):
    if pid_str.isdigit():
        pid = int(pid_str)
        if pid in ignore:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().decode("utf-8", "ignore")
            if target.search(cmd):
                os.kill(pid, signal.SIGKILL)
        except Exception:
            pass
' > /dev/null 2>&1 || true
    docker exec ros_container rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* > /dev/null 2>&1 || true
    sleep 0.5
}

# TTY detection for docker exec
DOCKER_IT="-i"
if [ -t 0 ]; then
    DOCKER_IT="-it"
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
    cleanup_workcell
    echo "➜ Starte MoveIt 2 Workcell mit RViz2 GUI..."
    docker exec $DOCKER_IT ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && ros2 launch vision_guided_bin_picking bin_picking.launch.py auto_orchestrate:=true"

elif [ "$CHOICE" == "2" ]; then
    cleanup_workcell
    echo "➜ Führe automatisierten Testlauf aus..."
    docker exec $DOCKER_IT ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && python3 /root/ros-projects/src/vision_guided_bin_picking/tools/test_bin_picking.py"

else
    echo "Beendet."
    exit 0
fi
