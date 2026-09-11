#!/usr/bin/env python3
"""Compile the production arming/idle/localization methods against a fake SDK.

No ROS master, DDS channel, or robot connection is created.
"""
from pathlib import Path
import subprocess
import tempfile


root = Path(__file__).resolve().parents[1]
source = (root / "src/go2_control/src/sdk_bridge.cpp").read_text()


def method(signature):
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


harness = r'''
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#define ROS_ERROR(...) ((void)0)
#define ROS_WARN(...) ((void)0)
#define ROS_INFO(...) ((void)0)
#define ROS_ERROR_THROTTLE(...) ((void)0)
#define ROS_WARN_THROTTLE(...) ((void)0)
#define ROS_INFO_THROTTLE(...) ((void)0)
#define ROS_DEBUG_THROTTLE(...) ((void)0)
namespace ros {
struct TimerEvent {};
double clock = 10.0;
struct WallTime {
  double value = 0;
  WallTime() = default;
  explicit WallTime(double x): value(x) {}
  static WallTime now() { return WallTime(clock); }
  bool isZero() const { return value == 0; }
  double toSec() const { return value; }
  WallTime operator-(const WallTime& rhs) const { return WallTime(value-rhs.value); }
};
using SteadyTime = WallTime;
}
namespace std_msgs { struct Bool {
  using ConstPtr = std::shared_ptr<const Bool>;
  bool data;
}; }
namespace geometry_msgs {
struct Vec3 { double x=0, y=0, z=0; };
struct Twist { Vec3 linear, angular; };
}
template<class T> std::string asString(T x) { return std::to_string(x); }
namespace unitree { namespace robot { namespace go2 {
constexpr int ROBOT_SPORT_API_ID_CLASSICWALK = 2049;
struct SportClient {
  int api = 0;
  std::string payload;
  int Call(int id, const std::string& parameter, std::string& response) {
    api = id; payload = parameter; response = "robot rejection"; return -1;
  }
};
}}}
CLASSICCLIENT;
struct FakeSport {
  int classic_result = 0, stop_result = 0, stop_count = 0, classic_count = 0;
  int requestClassicWalk(std::string& response) {
    ++classic_count; response = "server detail"; return classic_result;
  }
  int StopMove() { ++stop_count; return stop_result; }
  int move_count = 0, move_result = 0, nonzero_count = 0;
  int Move(float x, float y, float z) {
    assert(y == 0);
    if (x != 0 || z != 0) ++nonzero_count;
    ++move_count; return move_result;
  }
};
struct Bridge {
  std::unique_ptr<FakeSport> sport_client_{new FakeSport};
  bool idle_stop_sent_ = true, mode_ok = true;
  std::string last_gait_error_, current_motion_mode_ = "mcf";
  std::string gait_policy_ = "preserve_current", last_classic_result_, last_classic_response_;
  std::atomic<bool> localization_ok_{true}, enabled_{true}, commanded_motion_active_{true};
  std::atomic<double> last_nonzero_command_wall_{1}, commanded_vx_{0.3}, commanded_wz_{0.2};
  std::atomic<int> last_move_result_{0};
  std::atomic<bool> no_step_response_{false};
  std::atomic<double> motion_command_start_{0}, last_motion_response_wall_{0};
  std::atomic<double> last_foot_unload_wall_{0}, joint_motion_ema_{0};
  double command_timeout_sec_=0.5, motion_episode_reset_gap_sec_=1.0;
  double motion_response_timeout_sec_=6.0;
  ros::WallTime last_cmd_wall_stamp_;
  double shapeForwardVelocity(double x) { return x; }
  double shapeYawRate(double x) { return x; }
  bool have_cmd_ = true;
  bool stop_retry_pending_ = false;
  ros::SteadyTime normal_stop_started_, stationary_since_, last_zero_send_;
  std::mutex stop_feedback_mutex_;
  double stop_feedback_rx_ = 0, stop_planar_speed_ = 0, stop_yaw_speed_ = 0;
  void publishControlEnabled(bool value) { assert(value == enabled_); }
  std::mutex cmd_mutex_;
  geometry_msgs::Twist last_cmd_;
  bool ensureRequiredMotionMode() { return mode_ok; }
  bool verifyRequiredMotionMode() { return mode_ok; }
METHODS
};
int main() {
  DetailedSportClient client;
  std::string raw_response;
  assert(client.requestClassicWalk(raw_response) == -1);
  assert(client.api == 2049 && client.payload == "{\"data\":true}");
  assert(raw_response == "robot rejection");
  Bridge b;
  b.sport_client_->classic_result = -1;
  assert(b.prepareDirectMoveControl());
  assert(b.sport_client_->classic_count == 0);
  assert(b.last_classic_result_ == "not_requested");
  b.gait_policy_ = "request_classic_once";
  assert(!b.prepareDirectMoveControl());
  assert(b.sport_client_->classic_count == 1);
  assert(b.last_gait_error_.find("robot response") != std::string::npos);
  assert(b.last_classic_response_ == "server detail");
  b.sport_client_->classic_result = 0;
  assert(b.prepareDirectMoveControl());
  assert(b.sport_client_->classic_count == 2);
  b.sport_client_->classic_result = 42;
  assert(!b.prepareDirectMoveControl());
  assert(b.last_gait_error_.find("42") != std::string::npos);
  b.mode_ok = false;
  assert(!b.prepareDirectMoveControl());
  assert(b.sport_client_->classic_count == 3);

  b.holdZeroVelocity();
  assert(b.sport_client_->stop_count == 0 && b.sport_client_->move_count == 0);
  b.idle_stop_sent_ = false; // a movement segment occurred
  for (int i=0; i<400; ++i) {
    ros::clock = 10 + i * 0.005;
    b.stop_feedback_rx_ = ros::clock;
    b.holdZeroVelocity();
  }
  assert(b.idle_stop_sent_ && b.enabled_);
  assert(b.sport_client_->stop_count == 0);
  assert(b.sport_client_->move_count >= 5 && b.sport_client_->move_count <= 8);
  // Explicit/fault stops retain API 1003 and retry a failed response.
  b.idle_stop_sent_ = false;
  b.sport_client_->stop_result = 7;
  b.holdZeroVelocity(true);
  assert(!b.idle_stop_sent_); // failed stop must not be marked successful
  assert(b.stop_retry_pending_);
  b.sport_client_->stop_result = 0;
  b.holdZeroVelocity(true);
  assert(b.idle_stop_sent_);
  assert(!b.stop_retry_pending_);

  b.idle_stop_sent_ = false;
  b.localizationCallback(std::make_shared<const std_msgs::Bool>(std_msgs::Bool{false}));
  assert(b.enabled_ && !b.have_cmd_ && !b.commanded_motion_active_);
  assert(b.commanded_vx_ == 0 && b.commanded_wz_ == 0);
  int stops = b.sport_client_->stop_count;
  b.localizationCallback(std::make_shared<const std_msgs::Bool>(std_msgs::Bool{true}));
  assert(b.enabled_ && !b.have_cmd_ && b.sport_client_->stop_count == stops);
  b.stopRobot();
  assert(b.idle_stop_sent_);

  // Unacknowledged send success cannot substitute for measured standstill.
  for (int scenario=0; scenario<4; ++scenario) {
    Bridge f;
    f.idle_stop_sent_ = false;
    for (int i=0; i<230; ++i) {
      ros::clock = 20 + i * 0.005;
      f.stop_feedback_rx_ = scenario == 0 ? 19 : ros::clock;
      f.stop_planar_speed_ = scenario == 1 ? 0.2 : 0;
      f.stop_yaw_speed_ = scenario == 2 ? 0.3 : (scenario == 3 ? NAN : 0);
      f.holdZeroVelocity();
    }
    assert(!f.enabled_ && !f.have_cmd_);
    assert(f.sport_client_->stop_count == 1 && f.idle_stop_sent_);
  }
  Bridge f;
  f.idle_stop_sent_ = false;
  f.sport_client_->move_result = 3102;
  f.sport_client_->stop_result = 7;
  f.holdZeroVelocity();
  assert(!f.enabled_ && !f.have_cmd_ && !f.idle_stop_sent_);
  f.sport_client_->stop_result = 0;
  f.holdZeroVelocity(true);
  assert(f.idle_stop_sent_ && f.sport_client_->stop_count == 2);

  // Exercise the production control callback, not only its idle helper.
  Bridge c;
  c.commanded_motion_active_ = false;
  c.have_cmd_ = false;
  c.controlCallback({});
  assert(c.sport_client_->move_count == 0 && c.sport_client_->stop_count == 0);
  c.have_cmd_ = true;
  c.last_cmd_.linear.x = 0.3;
  c.last_cmd_wall_stamp_ = ros::WallTime::now();
  c.controlCallback({});
  assert(c.sport_client_->nonzero_count == 1 && !c.idle_stop_sent_);
  c.last_cmd_.linear.x = 0;
  for (int i=0; i<150; ++i) {
    ros::clock += 0.005;
    c.last_cmd_wall_stamp_ = ros::WallTime::now();
    c.stop_feedback_rx_ = ros::clock;
    c.controlCallback({});
  }
  assert(c.enabled_ && c.idle_stop_sent_ && !c.commanded_motion_active_);
  assert(c.sport_client_->stop_count == 0 && c.sport_client_->nonzero_count == 1);
  // Renewed movement followed by a command timeout must still force a stop.
  ros::clock += 1.0;
  c.last_cmd_.linear.x = 0.3;
  c.last_cmd_wall_stamp_ = ros::WallTime::now();
  c.controlCallback({});
  ros::clock += 0.6;
  c.controlCallback({});
  assert(c.sport_client_->stop_count == 1 && c.idle_stop_sent_);
  // A new velocity cannot jump ahead of a failed stop acknowledgement.
  c.sport_client_->stop_result = 7;
  c.stopRobot();
  const int moves = c.sport_client_->move_count;
  c.last_cmd_wall_stamp_ = ros::WallTime::now();
  c.controlCallback({});
  assert(c.sport_client_->move_count == moves && c.stop_retry_pending_);
  c.enabled_ = false;
  c.sport_client_->stop_result = 0;
  c.controlCallback({});
  assert(c.idle_stop_sent_ && !c.stop_retry_pending_);
  assert(c.sport_client_->move_count == moves);
}
'''
methods = "\n".join(method(signature) for signature in (
    "bool prepareDirectMoveControl()", "void holdZeroVelocity(",
    "void failNormalStop(",
    "void controlCallback(",
    "void stopRobot()", "void localizationCallback("))
with tempfile.TemporaryDirectory(prefix="go2-idle-test-") as directory:
    cpp = Path(directory) / "test.cpp"
    binary = Path(directory) / "test"
    cpp.write_text(harness.replace("METHODS", methods).replace(
        "CLASSICCLIENT", method("class DetailedSportClient")))
    subprocess.run(["g++", "-std=c++14", "-pthread", str(cpp), "-o", str(binary)],
                   check=True)
    subprocess.run([str(binary)], check=True)
print("PASS: passive arming, classic response detail, repeated zero with standstill confirmation, stale/moving/invalid feedback fallback, send failure, stop retry, localization recovery")
