#include <ros/ros.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <string>

class SystemMonitor {
 public:
  SystemMonitor() : nh_(), pnh_("~") {
    pnh_.param<std::string>("mode", mode_, "mapping");
    pnh_.param("timeout_sec", timeout_, 1.0);
    lidar_sub_ = nh_.subscribe("/livox/lidar", 1, &SystemMonitor::lidarCallback, this);
    imu_sub_ = nh_.subscribe("/livox/imu", 10, &SystemMonitor::imuCallback, this);
    odom_sub_ = nh_.subscribe("/odom_nav", 10, &SystemMonitor::odomCallback, this);
    localization_sub_ = nh_.subscribe("/localization/ok", 10,
        &SystemMonitor::localizationCallback, this);
    control_enabled_sub_ = nh_.subscribe("/go2/control/enabled", 10,
        &SystemMonitor::controlEnabledCallback, this);
    ready_pub_ = nh_.advertise<std_msgs::Bool>("/go2/system/ready", 1, true);
    timer_ = nh_.createTimer(ros::Duration(0.5), &SystemMonitor::timerCallback, this);
  }

 private:
  bool fresh(const ros::WallTime& stamp, const ros::WallTime& now) const {
    return !stamp.isZero() && (now - stamp).toSec() <= timeout_;
  }
  void lidarCallback(const livox_ros_driver2::CustomMsg::ConstPtr&) { lidar_ = ros::WallTime::now(); }
  void imuCallback(const sensor_msgs::Imu::ConstPtr&) { imu_ = ros::WallTime::now(); }
  void odomCallback(const nav_msgs::Odometry::ConstPtr&) { odom_ = ros::WallTime::now(); }
  void localizationCallback(const std_msgs::Bool::ConstPtr& msg) { localization_ok_ = msg->data; }
  void controlEnabledCallback(const std_msgs::Bool::ConstPtr& msg) { control_enabled_ = msg->data; }
  void timerCallback(const ros::TimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    const bool base_ready = fresh(lidar_, now) && fresh(imu_, now) && fresh(odom_, now);
    const bool navigation_ready = localization_ok_ && control_enabled_;
    const bool ready = base_ready && (mode_ != "navigation" || navigation_ready);
    std_msgs::Bool msg;
    msg.data = ready;
    ready_pub_.publish(msg);
    if (!ready) {
      ROS_WARN_THROTTLE(5.0,
          "GO2 %s waiting: lidar=%s imu=%s odom=%s localization=%s control=%s",
          mode_.c_str(), fresh(lidar_, now) ? "ok" : "wait",
          fresh(imu_, now) ? "ok" : "wait",
          fresh(odom_, now) ? "ok" : "wait",
          localization_ok_ ? "ok" : "wait",
          control_enabled_ ? "enabled" : "disabled");
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber lidar_sub_, imu_sub_, odom_sub_, localization_sub_;
  ros::Subscriber control_enabled_sub_;
  ros::Publisher ready_pub_;
  ros::Timer timer_;
  std::string mode_;
  double timeout_ = 1.0;
  ros::WallTime lidar_, imu_, odom_;
  bool localization_ok_ = false;
  bool control_enabled_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_system_monitor");
  SystemMonitor monitor;
  ros::spin();
  return 0;
}
