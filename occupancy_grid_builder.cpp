/*
Nodo "occupancy_grid_builder que:
  inicializa la grilla
  la actualiza
  publica /map

  index=(ycelda​×ancho_total)+xcelda

    | Valor | Significado |
    | ----- | ----------- |
    | `-1`  | Desconocido |
    | `0`   | Libre       |
    | `100` | Ocupado     |

*/
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class OccupancyGridBuilder : public rclcpp::Node {
public:
    OccupancyGridBuilder() : Node("occupancy_grid_builder") {
        // 1. Suscriptor para dimensiones: [ancho, largo, min_x, min_y]
        dims_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/map_dims", 10, std::bind(&OccupancyGridBuilder::dims_callback, this, std::placeholders::_1));

        // 2. Publisher del mapa base
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        // Timer para publicar el mapa cada segundo
        timer_ = this->create_wall_timer( //timer_ → publish_map() cada 1 s
            std::chrono::seconds(1), std::bind(&OccupancyGridBuilder::publish_map, this));

        RCLCPP_INFO(this->get_logger(), "Esperando dimensiones de CoppeliaSim...");
    }

private:
    void dims_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        /*
        Es llamada una sola vez
        Define:
            tamaño físico del mundo
            resolución del mapa
            origen del sistema de coordenadas
        Inicializa la memoria del mapa
        */
        if (map_initialized_) return;

        // Extraer datos
        float width_m = msg->data[0];
        float height_m = msg->data[1];
        origin_x_ = msg->data[2];
        origin_y_ = msg->data[3];

        // Calcular celdas (Resolución 0.05 = 20 celdas por metro)
        width_cells_ = static_cast<uint32_t>(width_m / resolution_); 
        height_cells_ = static_cast<uint32_t>(height_m / resolution_); // trunca, no redondea

        // Inicializar el vector del mapa con -1 (desconocido)
        grid_data_.assign(width_cells_ * height_cells_, -1); // equivale a: grid_data_.clear(); grid_data_.resize(width * height); for (...) grid_data_[i] = -1;

        RCLCPP_INFO(this->get_logger(), "Mapa inicializado: %u x %u celdas", width_cells_, height_cells_);
        map_initialized_ = true;
    }

    void publish_map() {
        if (!map_initialized_) return;

        auto map_msg = nav_msgs::msg::OccupancyGrid();
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map";

        // Info del mapa
        map_msg.info.resolution = resolution_;
        map_msg.info.width = width_cells_;
        map_msg.info.height = height_cells_;
        
        // El origen define que el (0,0) del mundo esté en el centro si origin_x/y son negativos
        map_msg.info.origin.position.x = origin_x_;
        map_msg.info.origin.position.y = origin_y_;
        map_msg.info.origin.orientation.w = 1.0;

        map_msg.data = grid_data_;

        map_pub_->publish(map_msg);
    }

    // Atributos
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr dims_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float resolution_ = 0.05;
    float origin_x_ = 0.0;
    float origin_y_ = 0.0;
    uint32_t width_cells_ = 0;
    uint32_t height_cells_ = 0; // unit32_t va de 0 a 2^{32}-1
    std::vector<int8_t> grid_data_; // int8_t va de -128 a 127
    bool map_initialized_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyGridBuilder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
