#include "slam/basalt/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory> // Required for std::shared_ptr

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<BasaltSLAMNode> node = std::make_shared<BasaltSLAMNode>();

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
