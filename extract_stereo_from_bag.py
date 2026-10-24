#!/mnt/fccedaf2-554d-47e5-8831-30c12ee6e08d/mechine/GPS_IMU_Fusion_EKF-main/.venv/bin/python3
"""
从 parking01.bag 提取图像（每3帧取1帧，可选仅提取左目）
使用 rosbags 库直接读取原始数据。

性能设计（35GB bag 单遍读取）：
- 不再做"先统计总数再提取"的两遍扫描；直接只迭代目标 topic 的连接。
- JPEG/PNG 起始标志用 bytes.find()（C 级）定位，替代逐字节 Python 循环。
"""

import struct
import sys
import cv2
import numpy as np
from pathlib import Path
from rosbags.rosbag1 import Reader

LEFT_TOPIC = '/avt_camera/left/image/compressed'
RIGHT_TOPIC = '/avt_camera/right/image/compressed'
IMU_TOPIC = '/insprobe/adis16465/imu'


def header_stamp(raw: bytes) -> float:
    """Parse the std_msgs/Header stamp out of a serialized ROS1 message.

    Layout: uint32 seq | int32 stamp.sec | uint32 stamp.nsec | uint32 str_len | str
    """
    _seq, sec, nsec = struct.unpack_from('<Iii', raw, 0)
    return sec + nsec * 1e-9


def first_message_time(reader, topic) -> float:
    """Time-stamp (s) of the first message on `topic`; used as the bag clock origin."""
    conns = [c for c in reader.connections if c.topic == topic]
    if not conns:
        raise RuntimeError(f"bag 中找不到 topic: {topic}")
    for _conn, _ts, raw in reader.messages(connections=conns):
        return header_stamp(raw)
    raise RuntimeError(f"topic {topic} 没有消息")


def extract_compressed_image(raw: bytes):
    """从 CompressedImage 原始字节中解码图像（JPEG/PNG）。"""
    for marker, _ in ((b'\xff\xd8\xff', 'jpeg'), (b'\x89PNG\r\n\x1a\n', 'png')):
        i = raw.find(marker)
        if i >= 0:
            img = cv2.imdecode(np.frombuffer(raw[i:], dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                return img
    return None


def _topic_connections(reader, wanted):
    """Return the connection objects whose topic is in `wanted`."""
    return [c for c in reader.connections if c.topic in wanted]


def extract_images(bag_file, output_dir, frame_interval=3, left_only=False):
    output_dir = Path(output_dir)
    left_dir = output_dir / 'left'
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir = None
    if not left_only:
        right_dir = output_dir / 'right'
        right_dir.mkdir(parents=True, exist_ok=True)

    wanted = {LEFT_TOPIC} if left_only else {LEFT_TOPIC, RIGHT_TOPIC}

    print(f"打开bag文件: {bag_file}")
    print(f"每 {frame_interval} 帧取1帧；目标topic: {sorted(wanted)}")

    # 时间基准：以 bag 内 IMU 首帧为 0，使图像时间戳与导出的 IMU txt 时钟一致
    # （parking01_ADIS16465_IMU.txt 与 bag 的 /insprobe/adis16465/imu 样本数、
    #   时长完全相同，即同一物理起点）
    with Reader(bag_file) as reader:
        t_imu0 = first_message_time(reader, IMU_TOPIC)
    print(f"bag 内 IMU 首帧时间戳: {t_imu0:.4f} s（作为时间原点）")
    print("单遍提取中...")

    left_seen = right_seen = 0
    left_saved = right_saved = 0
    last_report = 0
    stamps = []  # (frame_name, t_rel)

    with Reader(bag_file) as reader:
        conns = _topic_connections(reader, wanted)
        for conn, _timestamp, raw in reader.messages(connections=conns):
            if conn.topic == LEFT_TOPIC:
                if left_seen % frame_interval == 0:
                    img = extract_compressed_image(raw)
                    if img is not None:
                        name = f"frame_{left_seen:06d}.png"
                        cv2.imwrite(str(left_dir / name), img)
                        stamps.append((name, header_stamp(raw) - t_imu0))
                        left_saved += 1
                left_seen += 1
            else:  # RIGHT_TOPIC
                if right_seen % frame_interval == 0:
                    img = extract_compressed_image(raw)
                    if img is not None:
                        name = f"frame_{right_seen:06d}.png"
                        cv2.imwrite(str(right_dir / name), img)
                        right_saved += 1
                right_seen += 1

            if left_saved + right_saved - last_report >= 400:
                last_report = left_saved + right_saved
                print(f"  已保存 左:{left_saved} 右:{right_saved}", flush=True)

    if stamps:
        ts_path = left_dir / 'timestamps.txt'
        with open(ts_path, 'w') as fh:
            fh.write("# frame_name  t_rel_seconds(相对bag内IMU首帧)\n")
            for name, t in stamps:
                fh.write(f"{name} {t:.6f}\n")
        dt = np.diff([s[1] for s in stamps])
        print(f"\n真实采样间隔: 均值 {dt.mean()*1000:.2f} ms -> {1/dt.mean():.4f} Hz")
        print(f"时间戳已写入: {ts_path}")

    print()
    print("=" * 60)
    print("提取完成！")
    print("=" * 60)
    print(f"左目图像: {left_saved} 张（遍历 {left_seen} 张）")
    if not left_only:
        print(f"右目图像: {right_saved} 张（遍历 {right_seen} 张）")
    print(f"输出目录: {output_dir}")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("用法: python extract_stereo_from_bag.py <bag_file> <output_dir> [frame_interval] [--left-only]")
        print("示例: python extract_stereo_from_bag.py parking01.bag ./stereo_images 3 --left-only")
        sys.exit(1)

    bag_file = sys.argv[1]
    output_dir = sys.argv[2]
    args = [a for a in sys.argv[3:] if a != '--left-only']
    frame_interval = int(args[0]) if args else 3
    left_only = '--left-only' in sys.argv[3:]

    extract_images(bag_file, output_dir, frame_interval, left_only)
