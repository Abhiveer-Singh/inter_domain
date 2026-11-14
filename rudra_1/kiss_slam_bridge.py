#!/usr/bin/env python3
"""Live Webots -> KISS-SLAM adapter. No ROS middleware is used."""
from __future__ import annotations
import argparse, os, socket, struct, time, traceback
from pathlib import Path
import numpy as np
from kiss_slam.slam import KissSLAM
from kiss_slam.config import load_config

MAGIC_IN = 0x4B495353
MAGIC_OUT = 0x4B535250
HEADER = struct.Struct('<IIId')
POINT = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('t','<f4')])
REPLY = struct.Struct('<II16d')


# Independent, best-effort visualization publisher.
VIS_SOCKET = "/tmp/rudra_slam_vis.sock"
VIS_HEADER = struct.Struct("<4sIdIIII")
VIS_MAGIC = b"RSLM"
VIS_MAX_PACKET = 60000


def _cap_rows(array, maximum):
    array = np.asarray(array)
    if len(array) <= maximum:
        return array
    indices = np.linspace(0, len(array) - 1, maximum, dtype=np.int64)
    return array[indices]


def publish_visualization(sock, slam, frame, sim_time, pose):
    """Publish a compact snapshot without waiting for the visualizer."""
    try:
        scan = _cap_rows(np.asarray(frame, dtype=np.float32), 1200)

        poses = np.asarray(slam.poses, dtype=np.float64)
        trajectory = poses[:, :3, 3].astype(np.float32) if poses.ndim == 3 else np.empty((0, 3), np.float32)
        trajectory = _cap_rows(trajectory, 1600)

        local_maps = list(slam.local_map_graph.local_maps())
        keyposes = np.asarray(
            [node.keypose[:3, 3] for node in local_maps],
            dtype=np.float32,
        ).reshape((-1, 3))
        keyposes = _cap_rows(keyposes, 500)

        # Closure IDs refer to local-map IDs. Convert them to keypose indices.
        id_to_index = {int(node.id): i for i, node in enumerate(local_maps)}
        closure_pairs = []
        for source_id, target_id in slam.get_closures():
            a = id_to_index.get(int(source_id))
            b = id_to_index.get(int(target_id))
            if a is not None and b is not None:
                closure_pairs.append((a, b))

        closures = np.asarray(
            closure_pairs[:500], dtype="<u4"
        ).reshape((-1, 2))

        pose = np.asarray(pose, dtype="<f8").reshape((4, 4))
        packet = bytearray(
            VIS_HEADER.pack(
                VIS_MAGIC, 1, float(sim_time),
                len(scan), len(trajectory), len(keyposes), len(closures)
            )
        )
        packet.extend(pose.tobytes(order="C"))
        packet.extend(np.asarray(scan[:, :3], dtype="<f4").tobytes())
        packet.extend(np.asarray(trajectory, dtype="<f4").tobytes())
        packet.extend(np.asarray(keyposes, dtype="<f4").tobytes())
        packet.extend(closures.tobytes())

        if len(packet) > VIS_MAX_PACKET:
            return

        sock.sendto(packet, VIS_SOCKET)

    except (BlockingIOError, FileNotFoundError, ConnectionRefusedError):
        # No viewer, or its receive queue is full: drop this visualization update.
        pass
    except Exception as exc:
        print(f"[VIS] publish skipped: {exc}", flush=True)

def _nm(s):
    try: return sum(1 for _ in s.local_map_graph.local_maps())
    except Exception: return -1

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--socket',default='/tmp/rudra_kiss_slam.sock')
    ap.add_argument('--config',default=str(Path(__file__).with_name('kiss_slam.yaml')))
    ap.add_argument('--output',default=str(Path(__file__).with_name('kiss_output')))
    args=ap.parse_args()
    out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    cfg=load_config(Path(args.config))
    slam=KissSLAM(cfg)

    # Diagnostic: inspect every loop-closure candidate returned by the detector.
    original_get_best_closure = slam.closer.detector.get_best_closure

    def logged_get_best_closure(query_idx, local_map,
                                voxel_means=None, voxel_normals=None):
        candidate = original_get_best_closure(
            query_idx, local_map, voxel_means, voxel_normals
        )
        map_count = len(list(slam.local_map_graph.local_maps()))
        print(
            f"[LOOP-CANDIDATE] query={query_idx} "
            f"source={candidate.source_id} "
            f"target={candidate.target_id} "
            f"inliers={candidate.number_of_inliers} "
            f"points={len(local_map)} maps={map_count}",
            flush=True
        )
        return candidate

    slam.closer.detector.get_best_closure = logged_get_best_closure
    try: os.unlink(args.socket)
    except FileNotFoundError: pass
    server=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
    server.bind(args.socket); os.chmod(args.socket,0o666); server.listen(1)
    print(f'[KISS-SLAM] waiting on {args.socket}',flush=True)
    conn,_=server.accept(); print('[KISS-SLAM] Webots connected',flush=True)
    scan_count=0; last_save=0.0
    trajectory=open(out/'trajectory.csv','w',buffering=1)
    trajectory.write('scan,sim_time,x,y,z,closures\n')
    vis_sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    vis_sock.setblocking(False)
    print(f"[VIS] publishing to {VIS_SOCKET}", flush=True)
    while True:
        try: packet=conn.recv(2_000_000)
        except ConnectionError: break
        if not packet: break
        if len(packet)<HEADER.size: continue
        magic,version,count,sim_time=HEADER.unpack_from(packet)
        if magic!=MAGIC_IN or version!=1 or count<30: continue
        expected=HEADER.size+count*POINT.itemsize
        if len(packet)!=expected: continue
        rec=np.frombuffer(packet,dtype=POINT,count=count,offset=HEADER.size)
        frame=np.column_stack((rec['x'],rec['y'],rec['z'])).astype(np.float64,copy=False)
        timestamps=rec['t'].astype(np.float64,copy=True)
        # Webots point acquisition times can be identical for a static scan;
        # KISS-ICP accepts zero-duration timestamps in that case.
        timestamps[~np.isfinite(timestamps)]=0.0
        timestamps -= timestamps.min() if timestamps.size else 0.0
        try:
            slam.process_scan(frame,timestamps)
            T=np.asarray(slam.local_map_graph.last_keypose @ slam.odometry.last_pose,dtype=np.float64)
            if T.shape!=(4,4) or not np.isfinite(T).all(): T=np.eye(4)
            closures=len(slam.get_closures())
            publish_visualization(vis_sock, slam, frame, sim_time, T)
            try: conn.send(REPLY.pack(MAGIC_OUT,closures,*T.reshape(-1).tolist()))
            except OSError: break
            scan_count+=1
            trajectory.write(f'{scan_count},{sim_time:.6f},{T[0,3]:.6f},{T[1,3]:.6f},{T[2,3]:.6f},{closures}\n')
            if time.monotonic()-last_save>5.0:
                last_save=time.monotonic()
                # Rebuild the global map from finalized local maps using their
                # current optimized keyposes, then append the active submap.
                try:
                    chunks=[]
                    for local_map in slam.local_map_graph.local_maps():
                        if local_map.pcd is None:
                            continue
                        local_points=local_map.pcd.point.positions.numpy()
                        T=np.asarray(local_map.keypose,dtype=np.float64)
                        chunks.append(local_points @ T[:3,:3].T + T[:3,3])
                    active=np.asarray(slam.voxel_grid.point_cloud())
                    if active.ndim==2 and active.shape[1]>=3 and len(active):
                        T=np.asarray(slam.local_map_graph.last_keypose @ slam.odometry.last_pose,dtype=np.float64)
                        chunks.append(active[:,:3] @ T[:3,:3].T + T[:3,3])
                    if chunks:
                        import open3d as o3d
                        pts=np.vstack(chunks)
                        cloud=o3d.geometry.PointCloud()
                        cloud.points=o3d.utility.Vector3dVector(pts)
                        o3d.io.write_point_cloud(str(out/'global_map.ply'),cloud,write_ascii=False)
                except Exception as e: print(f'[KISS-SLAM] global map snapshot skipped: {e}',flush=True)
            if scan_count%20==0: print(f'[KISS-SLAM] scans={scan_count} points={count} closures={closures} maps={_nm(slam)} pose=({T[0,3]:.2f},{T[1,3]:.2f})',flush=True)
        except Exception:
            traceback.print_exc()
            # A failed frame is reported with identity; keep the socket alive.
            try: conn.send(REPLY.pack(MAGIC_OUT,len(slam.get_closures()),*np.eye(4).reshape(-1).tolist()))
            except OSError: break
    trajectory.close(); vis_sock.close(); conn.close(); server.close()

if __name__=='__main__': main()
