#!/usr/bin/env python3
"""Read-only live viewer for kiss_output: 3D map, trajectory, current position."""
import sys, time
from pathlib import Path
import numpy as np, open3d as o3d

out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name('kiss_output')
vis = o3d.visualization.Visualizer()
vis.create_window('KISS-SLAM map', 1280, 800)
ro = vis.get_render_option(); ro.point_size = 2.0
ro.background_color = np.array([0.08, 0.08, 0.10])
cloud = o3d.geometry.PointCloud(); path = o3d.geometry.LineSet()
marker = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6)
for g in (cloud, path, marker): vis.add_geometry(g)
pos = np.zeros(3); ply_m = 0.0; last = 0.0; first = True

while vis.poll_events():
    vis.update_renderer()
    time.sleep(0.02)
    if time.time() - last < 1.0: continue
    last = time.time()
    try:
        m = (out / 'global_map.ply').stat().st_mtime
        if m != ply_m:
            p = np.asarray(o3d.io.read_point_cloud(str(out / 'global_map.ply')).points)
            if len(p):
                ply_m = m
                z = (p[:, 2] - p[:, 2].min()) / max(np.ptp(p[:, 2]), 1e-6)
                cloud.points = o3d.utility.Vector3dVector(p)
                cloud.colors = o3d.utility.Vector3dVector(np.c_[z, 0.4 + 0.4 * (1 - z), 1 - z])
                vis.update_geometry(cloud)
        t = np.loadtxt(out / 'trajectory.csv', delimiter=',', skiprows=1, ndmin=2)
        if len(t) > 1:
            P = t[:, 2:5]
            path.points = o3d.utility.Vector3dVector(P)
            path.lines = o3d.utility.Vector2iVector(np.c_[np.arange(len(P) - 1), np.arange(1, len(P))])
            path.colors = o3d.utility.Vector3dVector(np.tile([1.0, 0.8, 0.0], (len(P) - 1, 1)))
            vis.update_geometry(path)
            marker.translate(P[-1] - pos, relative=True); pos = P[-1].copy()
            vis.update_geometry(marker)
            if first and len(cloud.points): vis.reset_view_point(True); vc = vis.get_view_control(); vc.set_front([0, -0.6, 0.8]); vc.set_up([0, 0, 1]); vc.set_zoom(0.5); first = False
    except Exception:
        pass  # files may be mid-write; try again next second
vis.destroy_window()
