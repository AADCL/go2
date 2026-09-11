"""Replay only ground/nonground into a guard on an isolated ROS master.

No navigation, velocity, SDK, goal, or enable publishers are created.
"""
import argparse
import collections
import json
import os
import signal
import subprocess
import time

import rosbag
import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('workspace')
    parser.add_argument('output')
    parser.add_argument('--config', help='Config for the isolated replay node')
    parser.add_argument('--near-support-radius', type=float,
                        help='Override only the isolated replay guard radius')
    args = parser.parse_args()
    if os.environ.get('ROS_MASTER_URI') != 'http://localhost:11321':
        raise RuntimeError('Use isolated ROS_MASTER_URI=http://localhost:11321')
    rospy.init_node('terrain_replay_analysis', anonymous=True)
    config = args.config or os.path.join(args.workspace, 'src/go2_terrain/config/terrain_guard_go2.yaml')
    with open(config) as stream:
        params = yaml.safe_load(stream)
    if args.near_support_radius is not None:
        if not 0.35 < args.near_support_radius <= 1.5:
            raise ValueError('replay radius must be within the near-field plane fit')
        params['health']['near_support_radius_m'] = args.near_support_radius
    rospy.set_param('/terrain_replay_guard', params)
    counts = collections.Counter()
    gate_counts = collections.Counter()
    height_hold_samples = [0]
    heights, processing = [], []
    start = [None]

    def status(msg):
        if start[0] is None or time.monotonic() - start[0] < 2.0:
            return
        for item in msg.status:
            counts[item.message] += 1
            values = {v.key: v.value for v in item.values}
            gate_counts[values['health_gate_open']] += 1
            height_hold_samples[0] += values.get('height_outlier_held') == 'true'
            heights.append(float(values['estimated_sensor_height_m']))
            processing.append(float(values['processing_ms']))

    sub = rospy.Subscriber('/replay/status', DiagnosticArray, status)
    pubs = {name: rospy.Publisher('/replay/'+name, PointCloud2, queue_size=2)
            for name in ('ground', 'nonground')}
    with open(args.output+'.node.log', 'w') as log:
        process = subprocess.Popen([
            os.path.join(args.workspace, 'devel/lib/go2_terrain/go2_terrain_guard_node'),
            '__name:=terrain_replay_guard', 'ground:=/replay/ground',
            'nonground:=/replay/nonground', 'diagnostics:=/replay/status',
            'healthy:=/replay/healthy', 'obstacles:=/replay/obstacles',
            'clearing:=/replay/clearing'], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+10
            while not all(p.get_num_connections() for p in pubs.values()):
                if process.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('guard did not subscribe')
                time.sleep(.05)
            first_stamp = None
            with rosbag.Bag(args.bag) as bag:
                for topic, msg, _ in bag.read_messages(topics=[
                        '/terrain/patchwork_ground', '/terrain/patchwork_nonground']):
                    stamp = msg.header.stamp.to_sec()
                    if first_stamp is None:
                        first_stamp = stamp
                        start[0] = time.monotonic()
                    wait = start[0]+stamp-first_stamp-time.monotonic()
                    if wait > 0:
                        time.sleep(wait)
                    pubs[topic.rsplit('_',1)[-1]].publish(msg)
            time.sleep(.15)
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
    result = {'status_counts':dict(counts), 'samples':len(heights),
              'gate_counts':dict(gate_counts),
              'height_hold_samples':height_hold_samples[0],
              'near_support_radius_m':params['health']['near_support_radius_m'],
              'height_min':min(heights) if heights else None,
              'height_max':max(heights) if heights else None,
              'processing_max_ms':max(processing) if processing else None}
    with open(args.output, 'w') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
