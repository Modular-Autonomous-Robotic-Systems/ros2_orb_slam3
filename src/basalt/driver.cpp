#include <memory>  // Required for std::shared_ptr

#include "rclcpp/rclcpp.hpp"
#include "slam/basalt/node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<BasaltSLAMNode> node = std::make_shared<BasaltSLAMNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
