#!/usr/bin/env python3
"""Independent live Open3D viewer for the Rudra KISS-SLAM bridge."""

import csv
import os
import socket
import struct
import time
from pathlib import Path

import numpy as np
import open3d as o3d

BASE = Path(__file__).resolve().parent
SOCKET_PATH = "/tmp/rudra_slam_vis.sock"
MAP_PATH = BASE / "kiss_output" / "global_map.ply"
GT_PATH = BASE / "kiss_output" / "ground_truth.csv"
ALIGN_PATH = BASE / "kiss_output" / "slam_alignment.csv"

# Packet layout shared with kiss_slam_bridge.py:
# header, 4x4 pose, scan xyz, trajectory xyz, keyposes xyz, closure index pairs
HEADER = struct.Struct("<4sIdIIII")
POSE_BYTES = 16 * 8
MAGIC = b"RSLM"
MAX_PACKET = 65507


def vec3(data, count, offset):
    size = count * 3 * 4
    if count == 0:
        return np.empty((0, 3), dtype=np.float32), offset
    arr = np.frombuffer(data, dtype="<f4", count=count * 3, offset=offset)
    return arr.reshape((-1, 3)).copy(), offset + size


def parse_packet(data):
    if len(data) < HEADER.size + POSE_BYTES:
        return None

    magic, version, sim_time, ns, nt, nk, nc = HEADER.unpack_from(data)
    if magic != MAGIC or version != 1:
        return None

    required = HEADER.size + POSE_BYTES + 12 * (ns + nt + nk) + 8 * nc
    if required != len(data):
        return None

    offset = HEADER.size
    pose = np.frombuffer(data, dtype="<f8", count=16, offset=offset)
    pose = pose.reshape((4, 4)).copy()
    offset += POSE_BYTES

    scan, offset = vec3(data, ns, offset)
    trajectory, offset = vec3(data, nt, offset)
    keyposes, offset = vec3(data, nk, offset)

    if nc:
        closures = np.frombuffer(
            data, dtype="<u4", count=nc * 2, offset=offset
        ).reshape((-1, 2)).copy()
    else:
        closures = np.empty((0, 2), dtype=np.uint32)

    return {
        "time": sim_time,
        "pose": pose,
        "scan": scan,
        "trajectory": trajectory,
        "keyposes": keyposes,
        "closures": closures,
    }


def make_cloud(points):
    cloud = o3d.geometry.PointCloud()
    if len(points):
        cloud.points = o3d.utility.Vector3dVector(
            np.asarray(points, dtype=np.float64)
        )
    return cloud


def update_lines(line, points, color):
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    line.points = o3d.utility.Vector3dVector(points)
    line.lines = o3d.utility.Vector2iVector(
        [[i, i + 1] for i in range(max(0, len(points) - 1))]
    )
    line.colors = o3d.utility.Vector3dVector(
        [color] * max(0, len(points) - 1)
    )


def read_alignment():
    try:
        values = np.fromstring(ALIGN_PATH.read_text().strip(), sep=",")
        if values.size != 4 or not np.isfinite(values).all():
            return None
        return values
    except (OSError, ValueError):
        return None


def read_ground_truth(alignment):
    if alignment is None or not GT_PATH.exists():
        return np.empty((0, 3))

    rows = []
    try:
        with GT_PATH.open(newline="") as f:
            for row in csv.DictReader(f):
                x = float(row.get("x", "nan"))
                y = float(row.get("y", "nan"))
                z = float(row.get("z", "0") or 0)
                if np.isfinite([x, y, z]).all():
                    rows.append((x, y, z))
    except (OSError, ValueError, csv.Error):
        return np.empty((0, 3))

    points = np.asarray(rows, dtype=np.float64).reshape((-1, 3))
    if not len(points):
        return points

    # Convert GPS world coordinates into the KISS-SLAM coordinate frame.
    a0, ox, oy, oz = alignment
    dx = points[:, 0] - ox
    dy = points[:, 1] - oy
    c, sn = np.cos(a0), np.sin(a0)

    aligned = np.empty_like(points)
    aligned[:, 0] = c * dx + sn * dy
    aligned[:, 1] = -sn * dx + c * dy
    aligned[:, 2] = points[:, 2] - oz
    return aligned


def main():
    try:
        os.unlink(SOCKET_PATH)
    except FileNotFoundError:
        pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(SOCKET_PATH)
    sock.setblocking(False)

    vis = o3d.visualization.Visualizer()
    if not vis.create_window("Rudra | Live KISS-SLAM", 1280, 800):
        raise RuntimeError("Could not create Open3D window")

    map_cloud = make_cloud([])
    scan_cloud = make_cloud([])
    trajectory_line = o3d.geometry.LineSet()
    gt_line = o3d.geometry.LineSet()
    closure_line = o3d.geometry.LineSet()
    rover_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6
    )

    vis.add_geometry(map_cloud)
    vis.add_geometry(scan_cloud)
    vis.add_geometry(trajectory_line)
    vis.add_geometry(gt_line)
    vis.add_geometry(closure_line)
    vis.add_geometry(rover_axes)

    opt = vis.get_render_option()
    opt.point_size = 2.0
    opt.background_color = np.array([0.08, 0.08, 0.08])

    latest = None
    axes_pose = np.eye(4)
    map_mtime = -1
    gt_signature = None
    last_gt_read = 0.0
    last_status = 0.0
    last_pose = np.eye(4)

    print("[VIS] Waiting for KISS-SLAM updates at", SOCKET_PATH, flush=True)
    print("[VIS] Map:", MAP_PATH, flush=True)
    print("[VIS] Ground truth:", GT_PATH, flush=True)

    try:
        while vis.poll_events():
            # Drain queued datagrams; only render the newest state.
            while True:
                try:
                    packet = sock.recv(MAX_PACKET)
                except BlockingIOError:
                    break
                parsed = parse_packet(packet)
                if parsed is not None:
                    latest = parsed

            if latest is not None:
                last_pose = latest["pose"]

                scan_world = latest["scan"].astype(np.float64)
                if len(scan_world):
                    scan_world = (
                        scan_world @ last_pose[:3, :3].T
                        + last_pose[:3, 3]
                    )
                scan_cloud.points = o3d.utility.Vector3dVector(scan_world)
                vis.update_geometry(scan_cloud)

                update_lines(
                    trajectory_line,
                    latest["trajectory"],
                    [0.1, 0.8, 0.2],
                )
                vis.update_geometry(trajectory_line)

                # Closure pairs index the keypose array.
                keyposes = latest["keyposes"]
                edges = []
                for a, b in latest["closures"]:
                    if a < len(keyposes) and b < len(keyposes):
                        edges.extend((keyposes[a], keyposes[b]))

                closure_line.points = o3d.utility.Vector3dVector(
                    np.asarray(edges, dtype=np.float64).reshape((-1, 3))
                )
                closure_line.lines = o3d.utility.Vector2iVector(
                    [[i, i + 1] for i in range(0, len(edges), 2)]
                )
                closure_line.colors = o3d.utility.Vector3dVector(
                    [[1.0, 0.15, 0.1]] * (len(edges) // 2)
                )
                vis.update_geometry(closure_line)

                rover_axes.transform(np.linalg.inv(axes_pose))
                rover_axes.transform(last_pose)
                axes_pose = last_pose.copy()
                vis.update_geometry(rover_axes)

            now = time.monotonic()

            # Reload the global map only when the bridge replaces its PLY.
            try:
                mtime = MAP_PATH.stat().st_mtime
                if mtime != map_mtime:
                    new_map = o3d.io.read_point_cloud(str(MAP_PATH))
                    map_cloud.points = new_map.points
                    if new_map.has_colors():
                        map_cloud.colors = new_map.colors
                    vis.update_geometry(map_cloud)
                    map_mtime = mtime
            except (OSError, RuntimeError):
                pass

            # GPS is transformed into the SLAM frame for visual comparison only.
            if now - last_gt_read >= 0.5:
                last_gt_read = now
                gt_stat = GT_PATH.stat() if GT_PATH.exists() else None
                align_stat = ALIGN_PATH.stat() if ALIGN_PATH.exists() else None
                signature = (
                    gt_stat.st_mtime_ns if gt_stat else -1,
                    align_stat.st_mtime_ns if align_stat else -1,
                )
                if signature != gt_signature:
                    alignment = read_alignment()
                    gt_points = read_ground_truth(alignment)
                    update_lines(gt_line, gt_points, [0.2, 0.5, 1.0])
                    vis.update_geometry(gt_line)
                    gt_signature = signature

            if now - last_status >= 2.0:
                last_status = now
                if latest is None:
                    print("[VIS] Waiting for bridge data...", flush=True)
                else:
                    p = last_pose[:3, 3]
                    print(
                        f"[VIS] t={latest['time']:.2f}s "
                        f"scan={len(latest['scan'])} "
                        f"trajectory={len(latest['trajectory'])} "
                        f"closures={len(latest['closures'])} "
                        f"pose=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})",
                        flush=True,
                    )

            vis.update_renderer()
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        vis.destroy_window()
        sock.close()
        try:
            os.unlink(SOCKET_PATH)
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    main()
