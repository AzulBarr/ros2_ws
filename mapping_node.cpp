#include <rclcpp/rclcpp.hpp>
#include "mapping.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // ROS Client Library
    auto node = std::make_shared<OccupancyGridBuilder>();
    rclcpp::spin(node); // Entrar en el loop de eventos de ROS y no salir hasta que el nodo termine
    rclcpp::shutdown();
    return 0;
}
