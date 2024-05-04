#include "robot_odometry/odom_node.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "wheel_odometry");

    ros::NodeHandle nh_;

    ros::NodeHandle nh_private("~");

    bool publish_tf = true;
    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::string left_motor_topic;
    std::string right_motor_topic;
    float wheel_separation;
    float wheel_radius;
    nh_private.param("publish_tf", publish_tf, true);
    nh_private.param<std::string>("base_frame_id", base_frame_id_ , "base_footprint");
    nh_private.param<std::string>("odom_frame_id", odom_frame_id_ , "odom");
    nh_private.param<std::string>("left_motor_topic", left_motor_topic , "motor/left");
    nh_private.param<std::string>("right_motor_topic", right_motor_topic , "motor/right");
    nh_private.param<float>("wheel_separation", wheel_separation , 0.206);
    nh_private.param<float>("wheel_radius", wheel_radius , 0.065/2);
    
    odom_node odom(&nh_, publish_tf, odom_frame_id_, base_frame_id_, left_motor_topic, right_motor_topic, wheel_separation, wheel_radius);

    ros::AsyncSpinner spinner(2);
    if(spinner.canStart())
        spinner.start();
    ros::Rate loopRate(10);

    while (ros::ok())
    {
        odom.updateOdometry();
        loopRate.sleep();
    }

    return 0;
}

odom_node::odom_node(ros::NodeHandle *nh_,bool pub_tf ,std::string odom_ , std::string base_ , std::string left_motor, std::string right_motor, float wheel_sep, float wheel_rad):
    rpm_accumulator_l_(0),
    rpm_accumulator_r_(0),
    publish_tf(pub_tf),
    odom_frame_id_(odom_),
    base_frame_id_(base_),
    left_motor_topic(left_motor),
    right_motor_topic(right_motor),
    wheel_separation(wheel_sep),
    wheel_radius(wheel_rad)
{
    l_rpm_accumulator_size_ = 10;
    r_rpm_accumulator_size_ = 10;
    resetAccumulators();
    rpm_accumulator_l_.accumulate(0.0);
    rpm_accumulator_r_.accumulate(0.0);
    
    // resetAccumulators();
    odom_pub = nh_->advertise<nav_msgs::Odometry>("odom", 50);
    sub1 = nh_->subscribe("motor/left", 50, &odom_node::leftMotorCallback, this);
    sub2 = nh_->subscribe("motor/right", 50, &odom_node::rightMotorCallback, this);
    
    
    time_.init();
    odometry_.init(time_);
    odometry_.setWheelParams(wheel_separation, wheel_radius, wheel_radius);
    ROS_INFO("Updating... %d", publish_tf);
}

void odom_node::resetAccumulators(){
    rpm_accumulator_l_ = RollingMeanAccumulator(l_rpm_accumulator_size_);
    rpm_accumulator_r_ = RollingMeanAccumulator(r_rpm_accumulator_size_);
}

void odom_node::leftMotorCallback(const motor_msgs::motor &speedData){
    lefty_ = true;
    rpm_accumulator_l_.accumulate(speedData.speed);
    // left_speed_ = speedData.speed;
    // updateOdometry();
}

void odom_node::rightMotorCallback(const motor_msgs::motor &speedData){
    righty_ = true;
    rpm_accumulator_r_.accumulate(speedData.speed);
    // right_speed_ = speedData.speed;
    // updateOdometry();
}

void odom_node::updateOdometry(){
    left_speed_ = rpm_accumulator_l_.getRollingMean();
    right_speed_ = rpm_accumulator_r_.getRollingMean();
    // time_.now();
    odometry_.update(lefty_ ? left_speed_ : 0,righty_ ? right_speed_ : 0, time_);
    lefty_ = righty_ = false;

    auto odom = tf::createQuaternionFromYaw(odometry_.getHeading());
    odom_quat.w = odom.getW();
    odom_quat.x = odom.getX();
    odom_quat.y = odom.getY();
    odom_quat.z = odom.getZ();
    auto empty = tf::createQuaternionFromYaw(0);
    empty_quat.w = empty.getW();
    empty_quat.x = empty.getX();
    empty_quat.y = empty.getY();
    empty_quat.z = empty.getZ();
    
    if(publish_tf){
        tf_broadcast.header.frame_id = odom_frame_id_;
        tf_broadcast.child_frame_id = base_frame_id_;
        tf_broadcast.transform.translation.x = odometry_.getX();
        tf_broadcast.transform.translation.y = odometry_.getY();
        tf_broadcast.transform.translation.z = 0.0;
        tf_broadcast.transform.rotation = odom_quat;
        tf_broadcast.header.stamp = time_.now();

        robot_tf_broadcaster.sendTransform(tf_broadcast);
    }

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time_.now();
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose.position.x = odometry_.getX();
    odom_msg.pose.pose.position.y = odometry_.getY();
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    if (left_speed_ == 0 && right_speed_ == 0){
        odom_msg.pose.covariance[0] = 1e-9;  //x pos variance
        odom_msg.pose.covariance[7] = 1e-3;  // y Variance
        odom_msg.pose.covariance[14] = 1e6;  // z Variance
        odom_msg.pose.covariance[21] = 1e6;  // xrotation Variance
        odom_msg.pose.covariance[28] = 1e6;  // yrotation Variance
        odom_msg.pose.covariance[35] = 1e-9; // zrotation Variance
        odom_msg.twist.covariance[0] = 1e-9; // x Variance
        odom_msg.twist.covariance[7] = 1e-3; // y Variance
        odom_msg.twist.covariance[14] = 1e6; // z Variance
        odom_msg.twist.covariance[21] = 1e6; // xrotation Variance
        odom_msg.twist.covariance[28] = 1e6; // yrotation Variance
        odom_msg.twist.covariance[35] = 1e-9;// zrotation Variance
    }
    else{
        odom_msg.pose.covariance[0] = 1e-3;
        odom_msg.pose.covariance[7] = 1e-3;
        odom_msg.pose.covariance[14] = 1e6;
        odom_msg.pose.covariance[21] = 1e6;
        odom_msg.pose.covariance[28] = 1e6;
        odom_msg.pose.covariance[35] = zyaw_covariance;
        odom_msg.twist.covariance[0] = 1e-3;
        odom_msg.twist.covariance[7] = 1e-3;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = zyaw_covariance;
    }
    odom_msg.twist.twist.linear.x = odometry_.getLinear();
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = odometry_.getAngular();

    odom_pub.publish(odom_msg);
}