#include <cmath>
#include <cstddef>
#include <vector>

#include <ros/ros.h>
#include <ros/time.h>

namespace robot_odometry
{
    class Odometry
    {
    private:
        double x_;
        double y_;
        double heading_;

        double linear_;
        double angular_;
        double linear_sum_;
        double angular_sum_;
        std::vector<double> linear_buffer_;
        std::vector<double> angular_buffer_;

        double wheel_separation_;
        double left_wheel_radius_;
        double right_wheel_radius_;

        void integrateRungeKutta(double linear, double angular);
        void integrateExact(double linear, double angular);
        void accumulateLinear(double linear);
        void accumulateAngular(double angular);
        void resetAccumulators();

        double getLinearMean();
        double getAngularMean();

    public:
        Odometry(/* args */);
        ~Odometry() = default;

        ros::Time time_;

        void init(ros::Time time);
        bool setWheelParams(double wheel_separation, 
            double left_wheel_radius, double right_wheel_radius);

        double getX();
        double getY();
        double getHeading();
        double getLinear();
        double getAngular();

        bool update(double left_vel, double right_vel, ros::Time current_time);
        void resetOdometry();
    };
} // namespace robot_odometry