#include <ros/ros.h>
#include <ros/time.h>
#include "robot_odometry/odom.h"
// #include <diff_drive_controller/speed_limiter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <motor_msgs/motor.h>

void leftMotorCallback(const motor_msgs::motor speedData);
void rightMotorCallback(const motor_msgs::motor speedData);

void updateOdometry();

double zyaw_covariance = 0.03;
double left_speed_;
double right_speed_;
bool lefty_ = false;
bool righty_ = false;
bool publish_tf = true;

std::string odom_frame_id_;
std::string base_frame_id_;

ros::Time time_;
ros::NodeHandle nh_;
ros::Publisher odom_pub;

geometry_msgs::Quaternion odom_quat;
geometry_msgs::Quaternion empty_quat;
geometry_msgs::TransformStamped tf_broadcast;

tf::TransformBroadcaster robot_tf_broadcaster;

// diff_drive_controller::SpeedLimiter speed_limit_;
robot_odometry::Odometry odometry_;
