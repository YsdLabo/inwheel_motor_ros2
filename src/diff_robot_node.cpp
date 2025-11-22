#include <rclcpp/rclcpp.hpp>
#include "diff_robot_controller.hpp"

class DiffRobotNode : public rclcpp::Node
{
public:
    DiffRobotNode() : Node("diff_robot_node")
    {
        //auto self_shared_ptr = shared_from_this();
        //drc_ = std::make_unique<DiffRobotController>(shared_from_this());
    }

    void run()
    {
        drc_ = std::make_unique<DiffRobotController>(shared_from_this());
        drc_->run();
    }

private:
    std::unique_ptr<DiffRobotController> drc_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffRobotNode>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
