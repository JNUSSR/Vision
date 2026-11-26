#!/usr/bin/env python3
import math
import sys
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    if len(sys.argv) != 2:
        raise SystemExit(f"Usage: {sys.argv[0]} BAG_PATH")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    lidar_type = topic_types.get("/livox/lidar")
    imu_type = topic_types.get("/livox/imu")
    if lidar_type is None or imu_type is None:
        raise SystemExit("Bag must contain /livox/lidar and /livox/imu")

    lidar_message = get_message(lidar_type)
    imu_message = get_message(imu_type)
    lidar_count = 0
    imu_count = 0
    point_count = 0
    min_points = math.inf
    max_points = 0
    sample_clouds = 0
    finite_count = 0
    range_min = math.inf
    range_max = 0.0
    line_counts = Counter()
    tag_counts = Counter()
    fast_lio_candidates = 0

    while reader.has_next() and (lidar_count < 5 or imu_count < 5):
        topic, data, _ = reader.read_next()
        if topic == "/livox/lidar" and lidar_count < 5:
            message = deserialize_message(data, lidar_message)
            lidar_count += 1
            min_points = min(min_points, message.point_num)
            max_points = max(max_points, message.point_num)
            point_count += message.point_num
            for point in message.points:
                line_counts[point.line] += 1
                tag_counts[point.tag & 0x30] += 1
                if point.line < 4 and (point.tag & 0x30) in (0x00, 0x10):
                    fast_lio_candidates += 1
                coordinates = (point.x, point.y, point.z)
                if not all(math.isfinite(value) for value in coordinates):
                    continue
                distance = math.sqrt(sum(value * value for value in coordinates))
                finite_count += 1
                range_min = min(range_min, distance)
                range_max = max(range_max, distance)
            sample_clouds += 1
        elif topic == "/livox/imu" and imu_count < 5:
            deserialize_message(data, imu_message)
            imu_count += 1

    print(f"Sample messages: LiDAR {lidar_count}, IMU {imu_count}")
    print(f"Points in sampled clouds: {point_count}; per cloud: {min_points:g}..{max_points}")
    print(f"Sampled XYZ points: {finite_count} across {sample_clouds} clouds")
    print(f"FAST-LIO tag/line candidates (4 scan lines): {fast_lio_candidates}")
    print(f"Lines: {dict(sorted(line_counts.items()))}; tag groups: {dict(sorted(tag_counts.items()))}")
    if finite_count:
        print(f"Finite point range: {range_min:.3f}..{range_max:.3f} m")


if __name__ == "__main__":
    main()
