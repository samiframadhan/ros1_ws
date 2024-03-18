#include "robot_odometry/odom.h"

namespace robot_odometry
{
    Odometry::Odometry()
    : x_(0.0)
    , y_(0.0)
    , heading_(0.0)
    , linear_(0.0)
    , angular_(0.0)
    , wheel_separation_(0.0)
    , left_wheel_radius_(0.0)
    , right_wheel_radius_(0.0)
    , time_(0.0)
    {
    }

    void Odometry::init(ros::Time time){
        resetAccumulators();
        time_ = time;
    }

    bool Odometry::update(double left_v, double right_v, ros::Time curr_time){
        const double dt = curr_time.toSec() - time_.toSec();

        const double linear = (left_v + right_v) * 0.5;
        const double angular = (right_v - left_v) / wheel_separation_;

        integrateExact(linear, angular);

        accumulateAngular(angular/dt);
        accumulateLinear(linear/dt);

        linear_ = getLinearMean();
        angular_ = getAngularMean();

        return true;
    }

    bool Odometry::setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius){
        wheel_separation_ = wheel_separation;
        left_wheel_radius_ = left_wheel_radius;
        right_wheel_radius_ = right_wheel_radius;

        return true;
    }

    double Odometry::getX(){
        return x_;
    }

    double Odometry::getY(){
        return y_;
    }

    double Odometry::getHeading(){
        return heading_;
    }

    double Odometry::getLinear(){
        return linear_;
    }

    double Odometry::getAngular(){
        return angular_;
    }

    void Odometry::resetOdometry(){
        x_ = 0.0;
        y_ = 0.0;
        heading_ = 0.0;
    }

    void Odometry::resetAccumulators(){
        linear_buffer_.clear();
        angular_buffer_.clear();
    }

    void Odometry::accumulateAngular(double angular){
        if(angular_buffer_.size() < 4){
            angular_buffer_.insert(angular_buffer_.begin(), angular);
            angular_sum_ += angular;
        }
        else{
            angular_sum_ -= angular_buffer_.back();
            angular_buffer_.pop_back();
            angular_buffer_.insert(angular_buffer_.begin(), angular);
            angular_sum_ += angular;
        }
    }

    double Odometry::getAngularMean(){
        size_t data_count = angular_buffer_.size();
        return angular_sum_ / data_count;
    }

    void Odometry::accumulateLinear(double linear){
        if(linear_buffer_.size() < 4){
            linear_buffer_.insert(linear_buffer_.begin(), linear);
            linear_sum_ += linear;
        }
        else{
            linear_sum_ -= linear_buffer_.back();
            linear_buffer_.pop_back();
            linear_buffer_.insert(linear_buffer_.begin(), linear);
            linear_sum_ += linear;
        }
    }

    double Odometry::getLinearMean(){
        size_t data_count = linear_buffer_.size();
        return linear_sum_ / data_count;
    }

    void Odometry::integrateExact(double linear, double angular){
        if(fabs(angular) < 1e-6){
            integrateRungeKutta(linear, angular);
        }
        else{
            const double heading_old = heading_;
            const double r = linear/ angular;
            heading_ += angular;
            x_ += r * (sin(heading_) - sin(heading_old));
            y_ += -r * (cos(heading_) - cos(heading_old));
        }
    }

    void Odometry::integrateRungeKutta(double linear, double angular){
        const double direction = heading_ + angular * 0.5;

        // Runge-Kutta 2nd order integration
        x_ += linear * cos(direction);
        y_ += linear * sin(direction);
        heading_ += angular;
    }
} // namespace robot_odometry
