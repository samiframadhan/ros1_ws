#include <ros/ros.h>
#include <ros/time.h>
#include "robot_odometry/odom.h"
// #include <diff_drive_controller/speed_limiter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <motor_msgs/motor.h>
#include "RollingMean.hpp"

class odom_node
{
private:
    double zyaw_covariance = 0.03;
    double left_speed_;
    double right_speed_;
    bool lefty_ = false;
    bool righty_ = false;
    bool publish_tf = true;

    double wheel_separation = 0.0;
    double wheel_radius = 0.0;
    
    std::string odom_frame_id_;
    std::string base_frame_id_;

    std::string left_motor_topic;
    std::string right_motor_topic;

    ros::Time time_;
    ros::NodeHandle nh_;
    ros::Publisher odom_pub;

    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::Quaternion empty_quat;
    geometry_msgs::TransformStamped tf_broadcast;

    tf::TransformBroadcaster robot_tf_broadcaster;

    // diff_drive_controller::SpeedLimiter speed_limit_;
    using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<float>;
    RollingMeanAccumulator rpm_accumulator_r_;
    RollingMeanAccumulator rpm_accumulator_l_;

    size_t r_rpm_accumulator_size_ = 10;
    size_t l_rpm_accumulator_size_ = 10;

    robot_odometry::Odometry odometry_;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
public:
    void leftMotorCallback(const motor_msgs::motor &speedData);
    void rightMotorCallback(const motor_msgs::motor &speedData);

    void updateOdometry();
    void resetAccumulators();
    odom_node(ros::NodeHandle *nh, bool pub_tf ,std::string odom_ , std::string base_ , std::string left_motor, std::string right_motor, float wheel_sep, float wheel_rad);
    ~odom_node() {}
};