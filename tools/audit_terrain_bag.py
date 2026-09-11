"""Read-only analysis of recorded Patchwork output through the production probe."""
import argparse
import subprocess

import numpy as np
import rosbag
from sensor_msgs import point_cloud2


def fit(points):
    if len(points) < 12:
        return float('nan'), float('nan')
    a = np.column_stack((points[:, :2], np.ones(len(points))))
    w = np.ones(len(points))
    for _ in range(5):
        p = np.linalg.lstsq(a * np.sqrt(w[:, None]),
                            points[:, 2] * np.sqrt(w), rcond=None)[0]
        r = np.abs(points[:, 2] - a @ p)
        w = np.minimum(1., .03 / np.maximum(r, 1e-9))
    return -p[2], np.degrees(np.arctan(np.linalg.norm(p[:2])))


def main():
    args = argparse.ArgumentParser()
    args.add_argument('bag')
    args.add_argument('probe')
    opts = args.parse_args()
    rows = []
    proc = subprocess.Popen([opts.probe], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, text=True)
    try:
        with rosbag.Bag(opts.bag) as bag:
            for _, msg, _ in bag.read_messages(topics=['/terrain/patchwork_ground']):
                pts = np.array(list(point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True)))
                if not len(pts):
                    continue
                proc.stdin.write(str(len(pts)) + '\n')
                np.savetxt(proc.stdin, pts, fmt='%.9g')
                proc.stdin.flush()
                data = proc.stdout.readline().split()
                if not data:
                    raise RuntimeError('probe stopped')
                h, a, b, rmse = map(float, data[:4])
                ids = np.array(list(map(int, data[4:])))
                pids = (np.floor((pts[:, 1]+4)/.15).astype(int)*67 +
                        np.floor((pts[:, 0]+5)/.15).astype(int))
                near = np.linalg.norm(pts[:, :2], axis=1) < 1.5
                chosen = pts[near & np.isin(pids, ids)]
                raw_h, raw_slope = fit(chosen)
                all_h, all_slope = fit(pts[near])
                rows.append([msg.header.stamp.to_sec(), h, raw_h, all_h,
                             np.degrees(np.arctan(np.hypot(a,b))), raw_slope,
                             all_slope, len(ids), len(chosen)])
    finally:
        proc.stdin.close()
        proc.wait()
    values = np.array(rows)
    print('frames', len(values))
    print('columns: stamp grid_height raw_selected_height all_near_height '
          'grid_slope raw_selected_slope all_near_slope cells selected_points')
    for row in sorted(rows, key=lambda r: r[1])[:12]:
        print(' '.join('%.5f' % v for v in row))
    for k, name in [(1,'grid'), (2,'raw_selected'), (3,'all_near')]:
        v=values[:,k]
        print(name, 'finite',np.isfinite(v).sum(), 'low',np.sum(v<.43),
              'percentiles',np.nanpercentile(v,[0,10,50,90,100]))


if __name__ == '__main__':
    main()
