#!/usr/bin/env python3
"""Record a Livox bag and report whether both sensor streams are arriving."""

import signal
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu


TOPICS = ("/livox/lidar", "/livox/imu")
REPORT_INTERVAL = 1.0
NO_DATA_TIMEOUT = 5.0
EXPECTED_RATES = {"/livox/lidar": 10.0, "/livox/imu": 200.0}
LOW_RATE_RATIO = 0.7
RATE_WARMUP = 5.0


def recorded_counts(bag_path):
    counts = dict.fromkeys(TOPICS, 0)
    databases = sorted(bag_path.glob("*.db3"))
    if not databases:
        raise RuntimeError("没有找到录制数据库")
    for database in databases:
        with sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True) as connection:
            rows = connection.execute(
                "SELECT topics.name, COUNT(messages.id) FROM topics "
                "LEFT JOIN messages ON topics.id = messages.topic_id GROUP BY topics.name"
            )
            for name, count in rows:
                if name in counts:
                    counts[name] += count
    return counts


def main():
    if len(sys.argv) != 2:
        raise SystemExit(f"Usage: {sys.argv[0]} BAG_PATH")
    bag_path = Path(sys.argv[1]).expanduser()
    if bag_path.exists():
        print(f"录制路径已存在，请换一个名称：{bag_path}", file=sys.stderr)
        return 2

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("livox_record_monitor")
    counts = dict.fromkeys(TOPICS, 0)
    previous_counts = counts.copy()
    last_received = dict.fromkeys(TOPICS, None)
    def lidar_callback(_message):
        counts[TOPICS[0]] += 1
        last_received[TOPICS[0]] = time.monotonic()

    def imu_callback(_message):
        counts[TOPICS[1]] += 1
        last_received[TOPICS[1]] = time.monotonic()

    node.create_subscription(CustomMsg, TOPICS[0], lidar_callback, qos_profile_sensor_data, raw=True)
    node.create_subscription(Imu, TOPICS[1], imu_callback, qos_profile_sensor_data)
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    process = None
    try:
        process = subprocess.Popen(["ros2", "bag", "record", "-o", str(bag_path), *TOPICS])
        print("正在检查点云和 IMU 数据，每秒报告累计和新增消息数。按 Ctrl+C 停止录制。", flush=True)
        last_report = time.monotonic()
        started_at = last_report
        streams_ready_at = None
        next_report = last_report + REPORT_INTERVAL
        while process.poll() is None and not stopping:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if streams_ready_at is None and all(value is not None for value in last_received.values()):
                streams_ready_at = now
            if now < next_report:
                continue
            elapsed = now - last_report
            increments = {topic: counts[topic] - previous_counts[topic] for topic in TOPICS}
            rates = {topic: increments[topic] / elapsed for topic in TOPICS}
            print(
                f"【数据检查】已累计点云 {counts[TOPICS[0]]} 条、IMU {counts[TOPICS[1]]} 条；"
                f"新增点云 {increments[TOPICS[0]]} 条、"
                f"IMU {increments[TOPICS[1]]} 条（近 {elapsed:.1f} 秒）；"
                f"接收频率：点云 {rates[TOPICS[0]]:.1f} Hz、IMU {rates[TOPICS[1]]:.1f} Hz。",
                flush=True,
            )
            previous_counts = counts.copy()
            missing = [
                topic for topic in TOPICS
                if now - (last_received[topic] or started_at) >= NO_DATA_TIMEOUT
            ]
            if missing:
                print(
                    "【警告】最近 5 秒没有收到：" + "、".join(missing)
                    + "。请先检查驱动和连接，暂时不要开始移动。",
                    flush=True,
                )
            if streams_ready_at is not None and now - streams_ready_at >= RATE_WARMUP:
                for topic in TOPICS:
                    if topic not in missing and rates[topic] < EXPECTED_RATES[topic] * LOW_RATE_RATIO:
                        print(
                            f"【低频警告】{topic} 最近 {elapsed:.1f} 秒仅新增 {increments[topic]} 条，"
                            f"接收频率 {rates[topic]:.1f} Hz，低于预期 {EXPECTED_RATES[topic]:g} Hz 的 70%。"
                            "请检查数据流；实际写入数量将在停止后核对。",
                            flush=True,
                        )
            last_report = now
            next_report = now + REPORT_INTERVAL

        if stopping and process.poll() is None:
            print("正在等待录制程序保存数据，请稍候……", flush=True)
        return_code = process.wait()
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()

    if not bag_path.is_dir():
        return return_code or 1
    try:
        saved = recorded_counts(bag_path)
    except (OSError, sqlite3.Error, RuntimeError) as error:
        print(f"【警告】无法核对录制文件：{error}", file=sys.stderr)
        return return_code or 1
    print(f"【录制结果】点云 {saved[TOPICS[0]]} 条，IMU {saved[TOPICS[1]]} 条。", flush=True)
    if any(saved[topic] == 0 for topic in TOPICS):
        print("【录制失败】至少一种数据为 0，无法用于定位，请检查后使用新路径重新录制。", file=sys.stderr)
        return return_code or 1
    if return_code:
        print("【警告】录制程序异常退出，请检查上面的错误信息。", file=sys.stderr)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
