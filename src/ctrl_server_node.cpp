#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "quadcopter_ctrl/ctrl_server.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("CTRL_SERVER");
    ctrl_server ctrl_server_node(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
