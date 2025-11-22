#include <rclcpp/rclcpp.hpp>
#include "diff_robot_controller.hpp"

class DiffRobotNode : public rclcpp::Node
{
public:
    DiffRobotNode() : Node("diff_robot_node")
    {
        auto self_shared_ptr = shared_from_this();
        drc = std::make_shared<DiffRobotController>(self_shared_ptr);
    }

private:
    std::shared_ptr<DiffRobotController> drc;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffRobotNode>());
    rclcpp::shutdown();
    return 0;
}
