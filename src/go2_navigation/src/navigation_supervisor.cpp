#include <ros/ros.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

class NavigationSupervisor {
 public:
  NavigationSupervisor() : nh_(), pnh_("~") {
    localization_sub_ = nh_.subscribe("/localization/ok", 10,
        &NavigationSupervisor::localizationCallback, this);
    control_enabled_sub_ = nh_.subscribe("/go2/control/enabled", 10,
        &NavigationSupervisor::controlEnabledCallback, this);
    simple_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
        &NavigationSupervisor::simpleGoalCallback, this);
    action_goal_sub_ = nh_.subscribe("/move_base/goal", 1,
        &NavigationSupervisor::actionGoalCallback, this);
    ready_pub_ = nh_.advertise<std_msgs::Bool>("/navigation/ready", 1, true);
    validated_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/move_base/validated_goal", 1, false);
    cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    zero_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_nav", 1);
    clear_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    reset_service_ = pnh_.advertiseService("reset",
        &NavigationSupervisor::resetCallback, this);
    publishReady(false);
  }

 private:
  void publishReady(bool ready) {
    std_msgs::Bool msg;
    msg.data = ready;
    ready_pub_.publish(msg);
  }

  void cancelAndStop() {
    actionlib_msgs::GoalID cancel;
    cancel.stamp = ros::Time::now();
    cancel_pub_.publish(cancel);
    zero_pub_.publish(geometry_msgs::Twist());
  }

  bool ready() const {
    return localization_ok_ && have_control_state_ && control_enabled_;
  }

  void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!ready()) {
      ROS_ERROR_STREAM("Navigation goal rejected: localization_ok="
                       << (localization_ok_ ? "true" : "false")
                       << ", control_enabled="
                       << (control_enabled_ ? "true" : "false")
                       << ". Run 'run_go2 enable' and publish a fresh goal.");
      cancelAndStop();
      return;
    }

    validated_goal_pub_.publish(*msg);
    ROS_INFO_STREAM("Navigation goal accepted in frame '"
                    << msg->header.frame_id << "'.");
  }

  void actionGoalCallback(
      const move_base_msgs::MoveBaseActionGoal::ConstPtr&) {
    if (!ready()) {
      ROS_ERROR_THROTTLE(1.0,
          "move_base action goal received while navigation is not ready; cancelling it");
      cancelAndStop();
    }
  }

  void localizationCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (localization_ok_ && !msg->data) {
      ROS_ERROR("Localization lost: cancelling navigation goal");
      cancelAndStop();
    }
    localization_ok_ = msg->data;
    publishReady(ready());
  }

  void controlEnabledCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (have_control_state_ && control_enabled_ && !msg->data) {
      ROS_ERROR("GO2 control disabled: cancelling navigation goal");
      cancelAndStop();
    }
    control_enabled_ = msg->data;
    have_control_state_ = true;
    publishReady(ready());
  }

  bool resetCallback(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& response) {
    cancelAndStop();
    std_srvs::Empty clear;
    const bool cleared = clear_client_.call(clear);
    response.success = true;
    response.message = cleared
        ? "Goal cancelled, zero command sent and costmaps cleared"
        : "Goal cancelled and zero command sent; costmap service unavailable";
    ROS_WARN_STREAM(response.message);
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber localization_sub_;
  ros::Subscriber control_enabled_sub_;
  ros::Subscriber simple_goal_sub_;
  ros::Subscriber action_goal_sub_;
  ros::Publisher ready_pub_;
  ros::Publisher validated_goal_pub_;
  ros::Publisher cancel_pub_;
  ros::Publisher zero_pub_;
  ros::ServiceClient clear_client_;
  ros::ServiceServer reset_service_;
  bool localization_ok_ = false;
  bool control_enabled_ = false;
  bool have_control_state_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "go2_navigation_supervisor");
  NavigationSupervisor supervisor;
  ros::spin();
  return 0;
}
