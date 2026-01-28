#ifndef __MAPPING_NODE_H__
#define __MAPPING_NODE_H__

#include <vector>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class OccupancyGridBuilder : public rclcpp::Node {
    public:
        OccupancyGridBuilder();

        void dims_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        void publish_map();

    private:
        void update_cell_log_odds(float wx, float wy, float l_update);
        void update_occupancy_grid();
        float quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q);
        bool update_laser_tf(const rclcpp::Time &required_time);

        // Atributos
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr dims_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        //rclcpp::TimerBase::SharedPtr timer_;
        bool map_initialized_ = false;
        bool pose_received_ = false;
        geometry_msgs::msg::Pose current_pose_;

        float resolution_ = 0.05f; // metros por celda
        float origin_x_ = 0.0;
        float origin_y_ = 0.0;
        uint32_t width_cells_ = 0;
        uint32_t height_cells_ = 0; // unit32_t va de 0 a 2^{32}-1
        std::vector<int8_t> grid_data_; // Valores int8 para ROS, int8_t va de -128 a 127
        std::vector<float> log_odds_grid_;  // Valores flotantes internos
        
        // Constantes Log-Odds CHEQUEAR ESTO
        const float l_occ_ = 0.85f;   // Lo que sumamos al detectar algo
        const float l_free_ = -0.40f; // Lo que restaríamos en espacio libre

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
        tf2::Transform laser_transform;
        bool transform_received;

        std::string robot_frame, laser_frame, publish_robot_frame;
};

#endif
