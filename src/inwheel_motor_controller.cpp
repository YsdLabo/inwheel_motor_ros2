#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist_stamped.hpp>
#include"inwheel_motor_driver.hpp"

#define WHEEL_RADIUS 0.070

class InwheelMotorController : public rclcpp::Node
{
public:
    InwheelMotorController() : Node("inwheel_motor_controller_node")
    {
        auto device = this->declare_parameter<std::string>("device", "/dev/ttyAMA0");
        motor_ = std::make_unique<InwheelMotorDriver>(device.c_str());
        motor_->motor_stop();
        motor_->reset_pulse_count();
        mps_to_rpm_ = 60.0 / (2.0 * M_PI * WHEEL_RADIUS);

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "cmd_vel",
            1,
            std::bind(&InwheelMotorController::callback_cmd_vel, this, std::placeholders::_1)
        );
    }
private:
    std::unique_ptr<InwheelMotorDriver>  motor_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
    double mps_to_rpm_;

    void callback_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        double rpm = msg->twist.linear.x * mps_to_rpm_;
        motor_->move(rpm);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InwheelMotorController>());
    rclcpp::shutdown();
    return 0;
}
