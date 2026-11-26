#!/usr/bin/env python3
"""Run the existing LiDAR/IMU pipelines and show relative positions in cm."""

import argparse
from collections import deque
import csv
from dataclasses import dataclass, field
import fcntl
from functools import partial
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu


PROJECT = Path(__file__).resolve().parent.parent
ALGORITHMS = {
    "fastlio": "FAST-LIO 原版",
    "fastlivo2": "FAST-LIVO2 雷达＋IMU",
}


@dataclass
class CoordinateState:
    origin: tuple = None
    position_cm: tuple = None
    frame: str = ""
    invalid: bool = False
    updated_at: float = None
    sensor_time: float = None
    received: deque = field(default_factory=lambda: deque(maxlen=1000))

    def update(self, message, now):
        position = message.pose.pose.position
        values = (position.x, position.y, position.z)
        self.invalid = not all(math.isfinite(value) for value in values)
        if self.invalid:
            return
        if self.origin is not None and message.header.frame_id != self.frame:
            self.invalid = True
            return
        if self.origin is None:
            self.origin = values
            self.frame = message.header.frame_id
        self.position_cm = tuple((value - origin) * 100
                                 for value, origin in zip(values, self.origin))
        self.sensor_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.updated_at = now
        self.received.append(now)


def frequency(samples, now):
    while samples and now - samples[0] > 2.0:
        samples.popleft()
    if len(samples) < 2:
        return 0.0
    return (len(samples) - 1) / max(now - samples[0], 0.001)


class CoordinateDisplay(Node):
    def __init__(self, algorithms):
        super().__init__("mid360_live_coordinates")
        self.states = {name: CoordinateState() for name in algorithms}
        self.inputs = {"点云": deque(maxlen=1000), "IMU": deque(maxlen=1000)}
        self.latest_imu_time = None
        self.subscriptions_owned = []
        for name, state in self.states.items():
            self.subscriptions_owned.append(self.create_subscription(
                Odometry, f"/mid360_live/{name}/odom",
                lambda message, current=state: current.update(message, time.monotonic()),
                qos_profile_sensor_data,
            ))
        self.subscriptions_owned.append(self.create_subscription(
            CustomMsg, "/livox/lidar", partial(self.receive_input, "点云"),
            qos_profile_sensor_data, raw=True,
        ))
        self.subscriptions_owned.append(self.create_subscription(
            Imu, "/livox/imu", self.receive_imu, qos_profile_sensor_data,
        ))

    def receive_input(self, label, _message):
        self.inputs[label].append(time.monotonic())

    def receive_imu(self, message):
        self.receive_input("IMU", message)
        self.latest_imu_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def sensor_lag(self, state, now):
        if (self.latest_imu_time is None or state.sensor_time is None
                or not self.inputs["IMU"] or now - self.inputs["IMU"][-1] >= 1.0):
            return None
        return self.latest_imu_time - state.sensor_time

    def lines(self, now):
        lines = ["MID360S 实时坐标   单位 cm", ""]
        for label, samples in self.inputs.items():
            age = now - samples[-1] if samples else math.inf
            rate = frequency(samples, now)
            status = f"{rate:.1f} Hz" if age < 1.0 else "等待数据 / 数据中断"
            lines.append(f"{label}输入   {status}")
        for name, state in self.states.items():
            lines.extend(["", ALGORITHMS[name]])
            age = now - state.updated_at if state.updated_at is not None else math.inf
            rate = frequency(state.received, now)
            if state.invalid:
                status = "坐标无效或坐标系变化，请停止后重新启动"
            elif state.origin is None:
                status = "等待首帧定位，请保持雷达静止"
            elif age >= 1.0:
                status = f"定位更新中断，以下为上次坐标，已过去 {age:.1f} s"
            else:
                status = f"更新中   {rate:.1f} Hz   最近更新 {age:.2f} s 前"
            lines.append(status)
            if name == "fastlivo2":
                lag = self.sensor_lag(state, now)
                if lag is None:
                    lines.append("数据延迟   等待可比较的 IMU 和定位数据")
                elif lag < -0.5:
                    lines.append("数据延迟   时间戳异常，请检查雷达时间")
                else:
                    warning = "  注意：数据积压" if lag >= 1.0 else ""
                    lines.append(f"数据延迟   约 {max(0.0, lag):.2f} s{warning}")
            values = state.position_cm
            for index, axis in enumerate("XYZ"):
                value = f"{values[index]:+12.2f}" if values is not None else "          --"
                lines.append(f"  {axis}   {value} cm")
        lines.extend([
            "", "每套算法以各自首帧位置为原点，X/Y/Z 沿用算法初始化坐标轴。",
            "看到“更新中”后开始移动。Ctrl+C 退出，重新启动会重新置零。",
        ])
        if any(not samples for samples in self.inputs.values()):
            lines.append("等待输入时检查供电、192 网段网卡，并关闭 LivoxViewer2。")
        return lines


class Processes:
    def __init__(self, log_directory):
        self.log_directory = log_directory
        self.children = []
        self.environment = dict(os.environ, OMP_WAIT_POLICY="PASSIVE", GOMP_SPINCOUNT="0")

    def start(self, name, command):
        log_path = self.log_directory / f"{name}.log"
        with log_path.open("w") as log:
            environment = dict(self.environment)
            if name == "fastlivo2":
                environment["LIVO_SENSOR_ODOM_TIME"] = "1"
            process = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True,
            )
        self.children.append((name, process, log_path))

    def check(self):
        for name, process, log_path in self.children:
            if process.poll() is not None:
                tail = "\n".join(log_path.read_text(errors="replace").splitlines()[-12:])
                raise RuntimeError(f"{name} 已退出，代码 {process.returncode}\n{tail}\n日志 {log_path}")

    def stop(self):
        for sig, timeout in [(signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
            # A ros2 CLI wrapper may exit before its child. Signal the whole owned group.
            for _name, process, _path in reversed(self.children):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + timeout
            for _name, process, _path in self.children:
                try:
                    process.wait(timeout=max(0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
            alive = False
            for _name, process, _path in self.children:
                try:
                    os.killpg(process.pid, 0)
                    alive = True
                except ProcessLookupError:
                    pass
            if not alive:
                break


def mapper_command(name):
    if name == "fastlio":
        command = [
            "ros2", "run", "fast_lio", "fastlio_mapping", "--ros-args",
            "--params-file", str(PROJECT / "fast_lio_comparison/config/mid360.yaml"),
            "-p", "static_confidence.enabled:=false",
            "-p", "pcd_save.pcd_save_en:=false", "-p", "runtime_pos_log_enable:=false",
            "-p", "publish.path_en:=false", "-p", "publish.map_en:=false",
            "-p", "publish.scan_publish_en:=false", "-p", "publish.scan_bodyframe_pub_en:=false",
            "-r", f"/Odometry:=/mid360_live/{name}/odom",
        ]
    else:
        command = [
            "ros2", "run", "fast_livo", "fastlivo_mapping", "--ros-args",
            "--params-file", str(PROJECT / "fast_livo2_ros2_src/config/mid360_recorded.yaml"),
            "-p", "common.lid_topic:=/mid360_live/lidar_pc2",
            "-p", "common.img_en:=0", "-p", "imu.imu_en:=true",
            "-p", "pcd_save.pcd_save_en:=false", "-p", "evo.pose_output_en:=false",
            "-p", "image_save.img_save_en:=false",
            "-r", f"/aft_mapped_to_init:=/mid360_live/{name}/odom",
        ]
    command += ["-r", f"__node:=mid360_live_{name}"]
    for topic in ["/tf", "/tf_static", "/path", "/cloud_registered", "/cloud_registered_body"]:
        command += ["-r", f"{topic}:=/mid360_live/{name}{topic}"]
    return command


def main():
    parser = argparse.ArgumentParser(description="实时显示相对首帧的 X/Y/Z，单位 cm")
    parser.add_argument("legacy_algorithm", nargs="?", choices=["both", *ALGORITHMS],
                        help=argparse.SUPPRESS)
    parser.add_argument("--algorithm", choices=["lio", "livo2", "both"],
                        help="选择 FAST-LIO、FAST-LIVO2 雷达＋IMU 或同时运行，默认 both")
    parser.add_argument("--no-driver", action="store_true", help="使用已运行的驱动或外部数据源")
    args = parser.parse_args()
    if args.algorithm is not None and args.legacy_algorithm is not None:
        parser.error("算法选择只能指定一次，请使用 --algorithm lio、livo2 或 both")
    selected = args.algorithm or args.legacy_algorithm or "both"
    selected = {"lio": "fastlio", "livo2": "fastlivo2"}.get(selected, selected)
    algorithms = list(ALGORITHMS) if selected == "both" else [selected]
    lock = open(f"/tmp/mid360-live-{os.getuid()}-{os.environ.get('ROS_DOMAIN_ID', '0')}.lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("实时坐标程序已在运行，请先在原终端按 Ctrl+C 退出。", file=sys.stderr)
        return 2
    # Resolve installed packages before starting any child processes.
    for name in algorithms:
        get_package_prefix("fast_lio" if name == "fastlio" else "fast_livo")
    if "fastlivo2" in algorithms:
        get_package_prefix("mid360_live")
    log_directory = Path.home() / ".ros/log" / time.strftime("mid360_live_%Y%m%d_%H%M%S")
    log_directory = log_directory.with_name(f"{log_directory.name}_{os.getpid()}")
    log_directory.mkdir(parents=True)
    processes = Processes(log_directory)
    display = None
    stopping = False

    def stop_requested(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop_requested)
    signal.signal(signal.SIGTERM, stop_requested)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    interactive = sys.stdout.isatty()
    result = 0
    diagnostics = None
    try:
        display = CoordinateDisplay(algorithms)
        if "fastlivo2" in algorithms:
            diagnostics = (log_directory / "latency.csv").open("w", newline="")
            diagnostics_writer = csv.writer(diagnostics)
            diagnostics_writer.writerow(["elapsed_s", "imu_sensor_time", "livo_sensor_time", "sensor_lag_s", "odom_receive_age_s"])
            started_at = time.monotonic()
            next_diagnostic = started_at
        if not args.no_driver:
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline and not stopping:
                rclpy.spin_once(display, timeout_sec=0.1)
            if not (display.count_publishers("/livox/lidar") or display.count_publishers("/livox/imu")):
                processes.start("driver", ["ros2", "launch", "livox_ros_driver2", "msg_MID360s_launch.py"])
        if not stopping:
            if "fastlivo2" in algorithms:
                processes.start("converter", [
                    "ros2", "run", "mid360_live", "livox_cloud_converter",
                    "--ros-args", "-r", "/livox/lidar_pc2:=/mid360_live/lidar_pc2",
                    "-r", "__node:=mid360_live_converter",
                ])
            for name in algorithms:
                processes.start(name, mapper_command(name))
        if interactive:
            print("\033[?1049h\033[?25l", end="", flush=True)
        next_render = 0.0
        while not stopping and rclpy.ok():
            rclpy.spin_once(display, timeout_sec=0.05)
            now = time.monotonic()
            if diagnostics is not None and now >= next_diagnostic:
                state = display.states["fastlivo2"]
                diagnostics_writer.writerow([
                    round(now - started_at, 3), display.latest_imu_time, state.sensor_time,
                    display.sensor_lag(state, now),
                    now - state.updated_at if state.updated_at is not None else None,
                ])
                diagnostics.flush()
                next_diagnostic = now + 1.0
            if now >= next_render:
                processes.check()
                screen = "\n".join(display.lines(now))
                print(("\033[H\033[J" if interactive else "") + screen, flush=True)
                next_render = now + (0.2 if interactive else 1.0)
    except Exception as error:
        result = 1
        failure = str(error)
    finally:
        processes.stop()
        if diagnostics is not None:
            diagnostics.close()
        if display is not None:
            display.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if interactive:
            print("\033[?25h\033[?1049l", end="", flush=True)
        lock.close()
    if result:
        print(f"实时坐标启动或运行失败\n{failure}", file=sys.stderr)
    print(f"实时坐标已停止。运行日志 {log_directory}", flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())
