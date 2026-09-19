#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""从 Gazebo/SDF 世界文件离线生成 2D 占据栅格地图。

适用场景：仿真环境里场地是固定的（world0.world 这种由静态 box 拼出来的
世界），不需要跑 gmapping，直接把世界文件里的静态几何体“投影”成一张和
仿真完全一致的地图，AMCL / move_base 才能正常定位和规划。

仓库里原来的 maps/map_simulation*.png 只是占位图（全白 + 中间一个黑方块），
和 world0.world 完全对不上，这也是机器人定位不到、走不起来的原因之一。

用法：
    python scripts/world_to_map.py --world worlds/world0.world --out maps/world0

会生成三个文件：
    maps/world0.pgm   占据栅格（map_server 直接读这个，最稳）
    maps/world0.png   同一张图的 PNG（方便肉眼查看）
    maps/world0.yaml  map_server 配置，image 指向上面的 pgm

注意：yaml 里的 origin 是图像左下角像素对应的世界坐标，脚本会自动算好，
不要手改；改过世界以后重新跑一次脚本即可。
"""

import argparse
import math
import os
import struct
import sys
import zlib
import xml.etree.ElementTree as ET


OCCUPIED = 0        # 障碍
FREE = 254          # 可通行
UNKNOWN = 205       # 未知（围墙之外）


def parse_pose(text):
    """把 SDF 的 pose 文本解析成 [x, y, z, roll, pitch, yaw]。"""
    values = [float(v) for v in str(text or "").split()]
    while len(values) < 6:
        values.append(0.0)
    return values


def rotate(x, y, yaw):
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return (x * cos_y - y * sin_y, x * sin_y + y * cos_y)


def collect_boxes(world_path, model_filter, z_min, z_max):
    """收集世界里所有 box 几何体在 2D 平面上的投影（含朝向）。"""
    root = ET.parse(world_path).getroot()
    world = root if root.tag == "world" else root.find("world")
    if world is None:
        raise ValueError("no <world> element in %s" % world_path)

    boxes = []
    for model in world.findall("model"):
        model_name = model.get("name", "")
        if model_filter and model_name != model_filter:
            continue
        for link in model.findall("link"):
            pose_el = link.find("pose")
            link_pose = parse_pose(pose_el.text) if pose_el is not None else [0.0] * 6

            candidates = []
            for tag in ("collision", "visual"):
                for element in link.findall(tag):
                    size_el = element.find("geometry/box/size")
                    if size_el is None:
                        continue
                    size = [float(v) for v in size_el.text.split()]
                    if len(size) < 2:
                        continue
                    element_pose_el = element.find("pose")
                    local_pose = parse_pose(element_pose_el.text) if element_pose_el is not None else [0.0] * 6
                    candidates.append((size, local_pose))
                if candidates:
                    break  # collision 优先，没有 collision 才退回 visual

            for size, local_pose in candidates:
                height = size[2] if len(size) > 2 else 0.0
                z_center = link_pose[2] + local_pose[2]
                if z_center + height * 0.5 < z_min or z_center - height * 0.5 > z_max:
                    continue  # 不在激光平面高度范围内，忽略（例如地面）
                offset_x, offset_y = rotate(local_pose[0], local_pose[1], link_pose[5])
                boxes.append({
                    "name": model_name + "/" + str(link.get("name", "link")),
                    "cx": link_pose[0] + offset_x,
                    "cy": link_pose[1] + offset_y,
                    "yaw": link_pose[5] + local_pose[5],
                    "length": size[0],
                    "width": size[1],
                })
    if not boxes:
        raise ValueError("world 里没有找到任何 box 几何体")
    return boxes


def footprint_corners(box):
    half_l = box["length"] * 0.5
    half_w = box["width"] * 0.5
    corners = []
    for dx, dy in ((-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)):
        rx, ry = rotate(dx, dy, box["yaw"])
        corners.append((box["cx"] + rx, box["cy"] + ry))
    return corners


def rasterize(boxes, resolution, margin):
    """把 box 列表画成灰度栅格，返回 (rows, cols, data, origin_x, origin_y)。"""
    all_x = []
    all_y = []
    for box in boxes:
        for x, y in footprint_corners(box):
            all_x.append(x)
            all_y.append(y)

    min_x = min(all_x) - margin
    max_x = max(all_x) + margin
    min_y = min(all_y) - margin
    max_y = max(all_y) + margin

    cols = int(math.ceil((max_x - min_x) / resolution))
    rows = int(math.ceil((max_y - min_y) / resolution))
    origin_x = min_x
    origin_y = min_y

    prepared = []
    for box in boxes:
        prepared.append((box, math.cos(-box["yaw"]), math.sin(-box["yaw"])))

    data = bytearray(rows * cols)
    for row in range(rows):
        # 图像第 0 行对应世界坐标 y 最大的地方
        world_y = origin_y + (rows - row - 0.5) * resolution
        in_rect_y = min(all_y) <= world_y <= max(all_y)
        for col in range(cols):
            world_x = origin_x + (col + 0.5) * resolution
            value = FREE if in_rect_y and min(all_x) <= world_x <= max(all_x) else UNKNOWN
            for box, cos_y, sin_y in prepared:
                dx = world_x - box["cx"]
                dy = world_y - box["cy"]
                local_x = dx * cos_y - dy * sin_y
                local_y = dx * sin_y + dy * cos_y
                if abs(local_x) <= box["length"] * 0.5 and abs(local_y) <= box["width"] * 0.5:
                    value = OCCUPIED
                    break
            data[row * cols + col] = value

    return rows, cols, bytes(data), origin_x, origin_y


def write_pgm(path, rows, cols, data):
    with open(path, "wb") as handle:
        handle.write(b"P5\n")
        handle.write(b"# CREATOR: robot_vs world_to_map.py\n")
        handle.write(("%d %d\n" % (cols, rows)).encode("ascii"))
        handle.write(b"255\n")
        handle.write(data)


def write_png(path, rows, cols, data):
    """只用标准库写 8bit 灰度 PNG，方便肉眼查看。"""
    def chunk(tag, payload):
        return (struct.pack(">I", len(payload)) + tag + payload +
                struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    raw = bytearray()
    for row in range(rows):
        raw.append(0)  # filter type 0
        raw.extend(data[row * cols:(row + 1) * cols])

    header = struct.pack(">IIBBBBB", cols, rows, 8, 0, 0, 0, 0)
    body = (b"\x89PNG\r\n\x1a\n" +
            chunk(b"IHDR", header) +
            chunk(b"IDAT", zlib.compress(bytes(raw), 9)) +
            chunk(b"IEND", b""))
    with open(path, "wb") as handle:
        handle.write(body)


def write_yaml(path, image_name, resolution, origin_x, origin_y):
    lines = [
        "image: %s" % image_name,
        "resolution: %.6f" % resolution,
        "origin: [%.6f, %.6f, 0.000000]" % (origin_x, origin_y),
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.196",
        "",
    ]
    with open(path, "w") as handle:
        handle.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description="从 SDF world 生成 2D 占据栅格地图")
    parser.add_argument("--world", required=True, help="输入 .world 文件")
    parser.add_argument("--out", required=True, help="输出前缀，例如 maps/world0")
    parser.add_argument("--resolution", type=float, default=0.025, help="分辨率 m/px，默认 0.025")
    parser.add_argument("--margin", type=float, default=0.4, help="四周留白 m，默认 0.4")
    parser.add_argument("--model", default=None, help="只取指定 model，默认取全部")
    parser.add_argument("--z-min", type=float, default=0.02, help="激光平面高度下限")
    parser.add_argument("--z-max", type=float, default=1.0, help="激光平面高度上限")
    args = parser.parse_args()

    boxes = collect_boxes(args.world, args.model, args.z_min, args.z_max)
    rows, cols, data, origin_x, origin_y = rasterize(boxes, args.resolution, args.margin)

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    base = os.path.basename(args.out)

    pgm_path = os.path.join(out_dir, base + ".pgm")
    png_path = os.path.join(out_dir, base + ".png")
    yaml_path = os.path.join(out_dir, base + ".yaml")

    write_pgm(pgm_path, rows, cols, data)
    write_png(png_path, rows, cols, data)
    write_yaml(yaml_path, base + ".pgm", args.resolution, origin_x, origin_y)

    occupied = data.count(OCCUPIED)
    unknown = data.count(UNKNOWN)
    print("world   : %s" % args.world)
    print("boxes   : %d" % len(boxes))
    print("size    : %d x %d px  ( %.2f x %.2f m @ %.3f m/px )" % (
        cols, rows, cols * args.resolution, rows * args.resolution, args.resolution))
    print("origin  : [%.3f, %.3f]" % (origin_x, origin_y))
    print("pixels  : occupied=%d free=%d unknown=%d" % (
        occupied, len(data) - occupied - unknown, unknown))
    print("written : %s" % pgm_path)
    print("          %s" % png_path)
    print("          %s" % yaml_path)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("world_to_map failed: %s" % exc, file=sys.stderr)
        sys.exit(1)
