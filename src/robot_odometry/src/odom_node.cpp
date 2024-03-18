#include "robot_odometry/odom_node.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "wheel_odometry");

    uint error = 0;
    double wheel_separation = 0.0;
    double wheel_radius = 0.0;
    std::string left_motor_topic;
    std::string right_motor_topic;
    error += nh_.getParam("wheel_separation", wheel_separation);
    error += nh_.getParam("wheel_radius", wheel_radius);
    error += nh_.getParam("publish_tf", publish_tf);
    error += nh_.getParam("left_motor_topic", left_motor_topic);
    error += nh_.getParam("right_motor_topic", right_motor_topic);
    error += nh_.getParam("base_frame_id", base_frame_id_);
    error += nh_.getParam("odom_frame_id", odom_frame_id_);

    auto left_sub = nh_.subscribe("motor/left", 50, leftMotorCallback);
    auto right_sub = nh_.subscribe("motor/right", 50, rightMotorCallback);
    odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 50);

    time_.init();
    odometry_.init(time_);
    odometry_.setWheelParams(wheel_separation, wheel_radius, wheel_radius);

    ros::spin();
}

void leftMotorCallback(const motor_msgs::motor speedData){
    lefty_ = true;
    left_speed_ = speedData.speed;
    updateOdometry();
}

void rightMotorCallback(const motor_msgs::motor speedData){
    righty_ = true;
    right_speed_ = speedData.speed;
    updateOdometry();
}

void updateOdometry(){
    if(lefty_ && righty_){
        odometry_.update(left_speed_, right_speed_, time_);
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
            tf_broadcast.header.stamp = time_;

            robot_tf_broadcaster.sendTransform(tf_broadcast);
        }

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = time_;
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
}