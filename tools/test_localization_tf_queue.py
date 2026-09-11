#!/usr/bin/env python3
"""Exercise production TF timer wiring under a blocked NDT/main callback queue.

Starts a separate localhost ROS master; never connects to navigation or DDS.
Use --baseline to demonstrate the old shared-queue failure with the same load.
"""
import os
from pathlib import Path
import shlex
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

root = Path(__file__).resolve().parents[1]
source = (root / 'src/go2_localization/src/ndt_localizer.cpp').read_text()


def method(signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


start = source.index('ros::TimerOptions tfOptions(')
end = source.index('_tfSpinner.start();', start) + len('_tfSpinner.start();')
wiring = source[start:end]
if '--baseline' in sys.argv:
    wiring = wiring.replace('&_tfQueue);', 'nullptr);')

harness = r'''
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/bind.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <iostream>
class Localizer {
  ros::NodeHandle _nh;
  tf2_ros::TransformBroadcaster _br;
  ros::CallbackQueue _tfQueue;
  ros::AsyncSpinner _tfSpinner;
  std::mutex _tfMutex;
  ros::Timer _tfTimer;
  tf::Transform _odomMap;
  struct Config {
    std::string mapFrame = "map", odomFrame = "odom";
    double tfPostdateSec = 0.10;
  } _cfg;
  void publishHealth(bool) {}
  METHODS
public:
  Localizer(): _tfSpinner(1, &_tfQueue) {
    _odomMap.setIdentity();
    WIRING
  }
  ~Localizer() { _tfTimer.stop(); _tfSpinner.stop(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_queue_regression");
  ros::NodeHandle nh;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  Localizer localizer;
  // Warm up both TF transport and the baseline shared queue.
  auto warmup = ros::WallTime::now();
  while ((ros::WallTime::now()-warmup).toSec() < 1.0) {
    ros::spinOnce(); ros::WallDuration(0.005).sleep();
  }
  std::atomic<bool> running{true};
  std::atomic<int> checked{0}, failed{0}, blocked{0};
  std::thread monitor([&] {
    while(running) {
      try {
        buffer.lookupTransform("odom", "map", ros::Time::now());
      } catch (const tf2::TransformException&) { ++failed; }
      ++checked;
      ros::WallDuration(0.01).sleep();
    }
  });
  auto busy = nh.createTimer(ros::Duration(0.15), [&](const ros::TimerEvent&) {
    ++blocked;
    ros::WallDuration(0.35).sleep(); // NDT-like work on the production main queue
  });
  auto start = ros::WallTime::now();
  while ((ros::WallTime::now()-start).toSec() < 2.5) ros::spinOnce();
  busy.stop();
  running = false;
  monitor.join();
  std::cout << "TF checks=" << checked << " failed=" << failed
            << " blocking_callbacks=" << blocked << std::endl;
  return checked >= 100 && blocked >= 3 && failed == 0 ? 0 : 1;
}
'''

with tempfile.TemporaryDirectory(prefix='go2-tf-test-') as directory:
    folder = Path(directory)
    cpp = folder / 'tf_test.cpp'
    binary = folder / 'tf_test'
    cpp.write_text(harness.replace('METHODS', method('void tfTimerCallback(')
                                  + '\n' + method('void publishTF()'))
                   .replace('WIRING', wiring))
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs', 'roscpp', 'tf2_ros', 'tf'], text=True))
    subprocess.run(['g++', '-std=c++14', '-pthread', str(cpp), '-o', str(binary)]
                   + flags, check=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    env = os.environ.copy()
    env.pop('ROS_HOSTNAME', None)
    env.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
               ROS_HOME=str(folder / 'ros'), ROS_LOG_DIR=str(folder / 'logs'))
    with (folder / 'master.log').open('w') as log:
        master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                  stdout=log, stderr=subprocess.STDOUT)
        try:
            proxy = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
            for _ in range(100):
                try:
                    if proxy.getPid('/tf_queue_test')[0] == 1:
                        break
                except OSError:
                    time.sleep(0.1)
            else:
                raise RuntimeError('isolated ROS master did not start')
            result = subprocess.run([str(binary)], env=env, timeout=20)
            if '--baseline' in sys.argv:
                assert result.returncode == 1, 'baseline must reproduce TF failure'
                print('PASS: baseline reproduces TF failure under blocked main queue')
            else:
                result.check_returncode()
                print('PASS: production TF queue remains available while main queue blocks')
        finally:
            master.terminate()
            try:
                master.wait(timeout=10)
            except subprocess.TimeoutExpired:
                master.kill()
                master.wait()
