"""Exercise the full terrain guard's height deadline on an isolated ROS master.

Publishes synthetic floor clouds only. No navigation or control publishers.
"""
import argparse
import json
import os
import signal
import subprocess
import time

import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('workspace')
    parser.add_argument('output')
    args = parser.parse_args()
    if os.environ.get('ROS_MASTER_URI') != 'http://localhost:11321':
        raise RuntimeError('Use isolated ROS_MASTER_URI=http://localhost:11321')
    rospy.init_node('height_hold_check', anonymous=True)
    with open(os.path.join(args.workspace,
                           'src/go2_terrain/config/terrain_guard_go2.yaml')) as stream:
        params = yaml.safe_load(stream)
    rospy.set_param('/height_hold_guard', params)
    samples = []

    def status(message):
        for entry in message.status:
            samples.append({value.key: value.value for value in entry.values})

    subscriber = rospy.Subscriber('/height_hold/status', DiagnosticArray, status)
    ground = rospy.Publisher('/height_hold/ground', PointCloud2, queue_size=1)
    nonground = rospy.Publisher('/height_hold/nonground', PointCloud2, queue_size=1)
    fields = [PointField(name, index * 4, PointField.FLOAT32, 1)
              for index, name in enumerate(('x', 'y', 'z', 'intensity'))]

    def publish(height):
        header = Header(stamp=rospy.Time.now(), frame_id='terrain_sensor')
        points = [(i * .08, j * .08, -height, 0.0)
                  for i in range(-20, 21) for j in range(-20, 21)]
        ground.publish(point_cloud2.create_cloud(header, fields, points))
        nonground.publish(point_cloud2.create_cloud(header, fields, []))

    def stream(height, count):
        for _ in range(count):
            begin = time.monotonic()
            publish(height)
            time.sleep(max(0, .05 - (time.monotonic() - begin)))

    results = {}
    with open(args.output + '.node.log', 'w') as log:
        process = subprocess.Popen([
            os.path.join(args.workspace, 'devel/lib/go2_terrain/go2_terrain_guard_node'),
            '__name:=height_hold_guard', 'ground:=/height_hold/ground',
            'nonground:=/height_hold/nonground', 'diagnostics:=/height_hold/status',
            'healthy:=/height_hold/healthy', 'obstacles:=/height_hold/obstacles',
            'clearing:=/height_hold/clearing'], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while not (ground.get_num_connections() and nonground.get_num_connections()):
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('guard did not subscribe')
                time.sleep(.05)
            for scenario in ('sustained_low', 'input_stops_after_outlier'):
                stream(.50, 30)
                assert samples[-1]['health_gate_open'] == 'true', samples[-1]
                start = len(samples)
                if scenario == 'sustained_low':
                    stream(.30, 16)
                else:
                    publish(.30)
                    time.sleep(.8)
                observed = samples[start:]
                held = [s for s in observed if s['height_outlier_held'] == 'true']
                closed = [s for s in observed if s['health_gate_open'] == 'false']
                assert held, (scenario, 'height hold was not exercised', observed)
                assert closed, (scenario, 'gate did not close', observed)
                close_age = float(closed[0]['last_healthy_frame_age_sec'])
                # Allow scheduling jitter; the ordinary 0.20 s health timer alone
                # would close near 0.40 s in the stopped-input scenario.
                assert .25 <= close_age < .35, (scenario, close_age)
                first_closed = observed.index(closed[0])
                assert all(s['health_gate_open'] == 'false'
                           for s in observed[first_closed:]), scenario
                results[scenario] = {'passed': True,
                                     'first_closed_healthy_age_sec': close_age,
                                     'held_samples': len(held)}
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
    with open(args.output, 'w') as stream_file:
        json.dump(results, stream_file, indent=2)
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
