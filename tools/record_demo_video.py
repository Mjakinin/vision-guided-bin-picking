#!/usr/bin/env python3
"""
Automated High-Definition Demo Video Recorder for Vision-Guided Bin-Picking.
Runs MoveIt 2 & RViz2 in an offscreen Xvfb virtual frame buffer (:99),
records the complete Pick-and-Place sequence using FFmpeg, and encodes
both a modern web MP4 video and an animated GitHub README GIF.
"""

import os
import sys
import time
import json
import signal
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BinPickingRecorder(Node):
    def __init__(self):
        super().__init__('bin_picking_recorder')
        self.telemetry = None
        self.sub = self.create_subscription(
            String, '/bin_picking/telemetry', self._cb, 10
        )

    def _cb(self, msg: String):
        try:
            self.telemetry = json.loads(msg.data)
        except Exception:
            pass


def cleanup():
    # Kill any stale Xvfb, ffmpeg, or ROS nodes
    try:
        my_pid = os.getpid()
        import re
        target = re.compile(r"Xvfb|ffmpeg|rviz2|move_group|ros2_control_node|bin_picking_orchestrator|vision_detector|planning_scene")
        for pid_str in os.listdir("/proc"):
            if pid_str.isdigit():
                p = int(pid_str)
                if p not in (1, my_pid):
                    try:
                        with open(f"/proc/{p}/cmdline", "rb") as f:
                            cmd = f.read().decode("utf-8", "ignore")
                        if target.search(cmd) and "record_demo_video" not in cmd:
                            os.kill(p, signal.SIGKILL)
                    except Exception:
                        pass
    except Exception:
        pass
    subprocess.run("rm -f /tmp/.X99-lock /tmp/.X11-unix/X99", shell=True, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def main():
    print("=" * 78)
    print("   VISION-GUIDED BIN-PICKING — AUTOMATED DEMO VIDEO RECORDER")
    print("=" * 78)

    cleanup()

    media_dir = "/root/ros-projects/src/vision_guided_bin_picking/docs/media"
    os.makedirs(media_dir, exist_ok=True)
    raw_mp4 = "/tmp/raw_bin_picking_demo.mp4"
    if os.path.exists(raw_mp4):
        os.remove(raw_mp4)

    # 1. Start Xvfb Virtual Framebuffer (:99 at 1280x720)
    print("➜ [1/5] Starte Xvfb Virtual Display (:99, 1280x720)...")
    xvfb_proc = subprocess.Popen(
        ["Xvfb", ":99", "-screen", "0", "1280x720x24", "-ac"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(1.5)

    env = os.environ.copy()
    env["DISPLAY"] = ":99"
    env["PYTHONUNBUFFERED"] = "1"

    # 2. Start FFmpeg screen capture on :99.0
    print("➜ [2/5] Initialisiere FFmpeg HD Video Recorder...")
    ffmpeg_cmd = [
        "ffmpeg", "-y",
        "-f", "x11grab",
        "-video_size", "1280x720",
        "-framerate", "25",
        "-draw_mouse", "0",
        "-i", ":99.0",
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-pix_fmt", "yuv420p",
        raw_mp4
    ]
    ffmpeg_proc = subprocess.Popen(
        ffmpeg_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(1.0)

    # 3. Launch MoveIt 2 Workcell with RViz2 & Autonomous Orchestration
    print("➜ [3/5] Starte MoveIt 2 Panda Workcell, RViz2 & Autonomen Orchestrator...")
    launch_cmd = [
        "ros2", "launch", "vision_guided_bin_picking", "bin_picking.launch.py",
        "use_rviz:=true", "auto_orchestrate:=true"
    ]
    launch_proc = subprocess.Popen(
        launch_cmd,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    rclpy.init()
    recorder_node = BinPickingRecorder()

    print("➜ [4/5] Zeichne Pick-and-Place Sequenz auf (Telemetry Tracking)...")
    t0 = time.time()
    max_duration = 100.0
    completed = False
    last_state = ""

    try:
        while rclpy.ok() and (time.time() - t0 < max_duration):
            rclpy.spin_once(recorder_node, timeout_sec=0.2)
            if recorder_node.telemetry:
                st = recorder_node.telemetry.get("state", "")
                cycle = recorder_node.telemetry.get("cycle", 0)
                subaction = recorder_node.telemetry.get("subaction", "")
                successes = recorder_node.telemetry.get("successes", 0)

                if st != last_state:
                    print(f"   [{time.time()-t0:4.1f}s] FSM: {st:<22} | Cycle: {cycle} | Picked: {successes}/4 ({subaction})")
                    last_state = st

                if st == "COMPLETED_ALL" or successes >= 4:
                    print("   ✓ Alle Werkstücke platziert! Zeichne Homing auf...")
                    completed = True
                    time.sleep(3.0)  # Capture return to home pose
                    break
    finally:
        print("➜ [5/5] Beende Aufzeichnung & kodiere Videoformate...")
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            launch_proc.wait(timeout=5)
        except Exception:
            pass

        # Stop ffmpeg cleanly via SIGINT
        try:
            ffmpeg_proc.send_signal(signal.SIGINT)
            ffmpeg_proc.wait(timeout=10)
        except Exception:
            ffmpeg_proc.kill()

        try:
            xvfb_proc.terminate()
            xvfb_proc.wait(timeout=3)
        except Exception:
            xvfb_proc.kill()

        recorder_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not os.path.exists(raw_mp4) or os.path.getsize(raw_mp4) < 10000:
        print("✗ Fehler: Raw-Videodatei wurde nicht erfolgreich erzeugt!")
        return 1

    final_mp4 = os.path.join(media_dir, "bin_picking_demo.mp4")
    final_gif = os.path.join(media_dir, "bin_picking_demo.gif")

    # Encode optimized web-friendly MP4 (H.264, FastStart)
    print(f"➜ Optimiere MP4 Video -> {final_mp4}...")
    subprocess.run([
        "ffmpeg", "-y", "-i", raw_mp4,
        "-vf", "scale=960:-2",
        "-c:v", "libx264", "-crf", "22", "-preset", "medium",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        final_mp4
    ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Encode lightweight animated GIF for GitHub README preview (10 fps, 640px)
    print(f"➜ Erzeuge GitHub README GIF -> {final_gif}...")
    subprocess.run([
        "ffmpeg", "-y", "-i", raw_mp4,
        "-vf", "fps=10,scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse",
        "-loop", "0",
        final_gif
    ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("=" * 78)
    print(" ✓ VIDEOAUFZEICHNUNG ERFOLGREICH ABGESCHLOSSEN!")
    print(f"   • MP4: {final_mp4} ({os.path.getsize(final_mp4) / (1024*1024):.1f} MB)")
    print(f"   • GIF: {final_gif} ({os.path.getsize(final_gif) / (1024*1024):.1f} MB)")
    print("=" * 78)

    cleanup()
    return 0


if __name__ == '__main__':
    sys.exit(main())
