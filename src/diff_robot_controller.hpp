#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "inwheel_motor_driver.hpp"

#define  WHEEL_RADIUS  0.070
#define  WHEEL_SEPARATION  0.276
#define  PULSE_PER_ROTATE  4096.0

class DiffRobotController
{
public:
    DiffRobotController(rclcpp::Node::SharedPtr node_ptr)
        : node_(node_ptr)
    {
        dev_wheel_right_ = node_->declare_parameter<std::string>("dev_wheel_right", "/dev/ttyAMA1");
        dev_wheel_left_ = node_->declare_parameter<std::string>("dev_wheel_left", "/dev/ttyAMA2");

        motor_right_ = std::make_unique<InwheelMotorDriver>(dev_wheel_right_.c_str());
        motor_left_  = std::make_unique<InwheelMotorDriver>(dev_wheel_left_.c_str());

        wheel_radius_ = node_->declare_parameter<double>("wheel_radius", WHEEL_RADIUS);
        wheel_separation_  = node_->declare_parameter<double>("wheel_separation", WHEEL_SEPARATION);

        radian_per_ticks_ = 2.0 * M_PI / PULSE_PER_ROTATE;
        meter_per_ticks_ = wheel_radius_ * radian_per_ticks_;
        mps_to_rpm_ = 60.0 / (2.0 * M_PI * wheel_radius_);
    }

    void run()
    {
        motor_right_->motor_stop();
        motor_left_->motor_stop();
        motor_right_->reset_pulse_count();
        motor_left_->reset_pulse_count();
        rpm_l_ = 0.0;
        rpm_r_ = 0.0;

        sub_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel",
            1,
            std::bind(&DiffRobotController::callback_cmd_vel, this, std::placeholders::_1)
        );
        pub_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states",
            1
        );
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DiffRobotController::callback_timer, this)
        );
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    std::string dev_wheel_right_;
    std::string dev_wheel_left_;
    double wheel_radius_;
    double wheel_separation_;

    std::unique_ptr<InwheelMotorDriver> motor_right_;
    std::unique_ptr<InwheelMotorDriver> motor_left_;
    double radian_per_ticks_;
    double meter_per_ticks_;
    double mps_to_rpm_;
    double rpm_r_;
    double rpm_l_;

    void callback_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        double linear_x = msg->twist.linear.x;
        double angular_z = msg->twist.angular.z;
        double turning = angular_z * wheel_separation_ * 0.5;
        rpm_r_ = (linear_x + turning) * mps_to_rpm_;
        rpm_l_ = -(linear_x - turning) * mps_to_rpm_;
    }

    void callback_timer()
    {
        int count_r, count_l;
        if(motor_right_->get_pulse_count(&count_r)==0 and motor_left_->get_pulse_count(&count_l)==0)
        {
            double angle_r = count_r * radian_per_ticks_;
            double angle_l = - count_l * radian_per_ticks_;

            sensor_msgs::msg::JointState js;
            js.header.stamp = node_->now();
            js.name = {"wheel_right_joint", "wheel_left_joint"};
            js.position = {angle_r, angle_l};
            pub_joint_states_->publish(js);
        }
        motor_right_->move(rpm_r_);
        motor_left_->move(rpm_l_);
    }

};
