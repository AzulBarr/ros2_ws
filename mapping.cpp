/*
Nodo "mapping_node que:
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
#include "mapping.hpp"


OccupancyGridBuilder::OccupancyGridBuilder() : Node("mapping_node"), tf_buffer(this->get_clock()), tf_listener(tf_buffer), transform_received(false) {
    // Suscriptores
    dims_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/map_dims", 10, std::bind(&OccupancyGridBuilder::dims_callback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/robot_pose", 10, std::bind(&OccupancyGridBuilder::pose_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&OccupancyGridBuilder::scan_callback, this, std::placeholders::_1));

    // Publisher del mapa
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    //this->declare_parameter("robot_frame", std::string("base_link"));
    //this->declare_parameter("publish_robot_frame", std::string("base_link"));
    this->declare_parameter("laser_frame", std::string("laser"));
    this->declare_parameter("map_frame", std::string("map"));

    //this->get_parameter("robot_frame", robot_frame);
    //this->get_parameter("publish_robot_frame", publish_robot_frame);
    this->get_parameter("laser_frame", laser_frame);
    this->get_parameter("map_frame", map_frame);
    

    RCLCPP_INFO(this->get_logger(), "Esperando dimensiones de CoppeliaSim...");
}
// --- CALLBACKS ---
void OccupancyGridBuilder::dims_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
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
    height_cells_ = static_cast<uint32_t>(height_m / resolution_);

    // Inicializar grilla de Log-Odds (en 0.0 porque log(0.5/0.5) = 0)
    log_odds_grid_.assign(width_cells_ * height_cells_, 0.0f);
    
    // Grilla para ROS (visualización)
    grid_data_.assign(width_cells_ * height_cells_, 0); // equivale a: grid_data_.clear(); grid_data_.resize(width * height); for (...) grid_data_[i] = -1;
    /*for (size_t i = 10; i < 50; ++i){
        grid_data_[i] = -1; // Nunca observado
    }
    for (size_t i = 60; i < grid_data_.size(); ++i){
        grid_data_[i] = 100; // Nunca observado
    }*/

    map_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Mapa inicializado: %u x %u celdas", width_cells_, height_cells_);
}

void OccupancyGridBuilder::pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    current_pose_ = *msg;
    pose_received_ = true;
}

void OccupancyGridBuilder::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    /*
    El mensaje está en el marco del láser y en coordenadas polares
    Lo quiero pasar a coordenadas cartesianas en el marco del mapa
    */
    if (!map_initialized_ || !pose_received_) return;

    if (!update_laser_tf(msg->header.stamp))
    {
        RCLCPP_WARN(this->get_logger(), "%s -> %s transform not yet received", laser_frame.c_str(), robot_frame.c_str());
        return;
    }
    /* Convertir range,bearing a puntos cartesianos x,y,0.
    Descartando aquellas mediciones por fuera de los rangos validos */
    for (long unsigned int i = 0; i < msg->ranges.size(); i++){
        /* Utilizar la informacion del mensaje para filtrar y convertir */
        float range = msg->ranges[i];
        float range_min = msg->range_min;
        float range_max = msg->range_max;

        float angle_min = msg->angle_min; 
        float angle_increment = msg->angle_increment;
        /* convierto el punto en relacion al marco de referencia del laser al marco del robot */
        if (range <= range_max && range >= range_min){
            // CASO 1: DETECTÓ OBSTÁCULO
            tf2::Vector3 p(range * cos(angle_min + angle_increment * i), range * sin(angle_min + angle_increment * i), 0);
            p = transform_laser_world * p;
            update_cell_log_odds(p.x(), p.y(), l_occ_); // Sumamos probabilidad de ocupado
        } else {
            for (float r = range_min; r <= range_max; r += resolution_){
                tf2::Vector3 p(r * cos(angle_min + angle_increment * i), r * sin(angle_min + angle_increment * i), 0);
                p = transform_laser_world * p;
                update_cell_log_odds(p.x(), p.y(), l_free_); // Restamos probabilidad (espacio libre)
            }

            // CASO 2: NO DETECTÓ NADA (dist == -1)
            // Se marca espacio libre a lo largo del rayo.
            //float fx = (range_max * 0.5f) * cos(angle_min + angle_increment * i); // Ejemplo: punto medio
            //float fy = (range_max * 0.5f) * sin(angle_min + angle_increment * i);
            //tf2::Vector3 p(fx, fy, 0);
            //p = laser_transform * p;
            //update_cell_log_odds(p.x(), p.y(), l_free_); // Restamos probabilidad (espacio libre)
        }
    }
    // 3. Convertir Log-Odds a OccupancyGrid (0-100) y publicar
    update_occupancy_grid();
    publish_map();
}

// ENTENDER Y MODIFICAR SI ES NECESARIO
void OccupancyGridBuilder::update_cell_log_odds(float wx, float wy, float l_update) {
    // Convertir coordenadas de mundo a índices de grilla
    int index;
    bool valid = worldToIndex(wx, wy, index);
    if (!valid) return;
    else{
        // Sumar Log-Odds y aplicar límites 
        log_odds_grid_[index] += l_update;
        // (clamping)
        //if (log_odds_grid_[index] > 5.0f) log_odds_grid_[index] = 5.0f;
        //if (log_odds_grid_[index] < -5.0f) log_odds_grid_[index] = -5.0f;
        log_odds_grid_[index] = std::clamp(log_odds_grid_[index], -5.0f, 5.0f);

    }
}
// 0 = ocupado, -1 = libre, 100 = desconocido
void OccupancyGridBuilder::update_occupancy_grid(){
    constexpr float OCCUPIED_THRESH = 0.65f;
    constexpr float FREE_THRESH     = 0.35f;
    for (size_t i = 0; i < log_odds_grid_.size(); ++i){
        float lo = log_odds_grid_[i];
        // Nunca observado
        if (lo == 0.0f){
            grid_data_[i] = 100;//-1
            continue;
        }
        // Log-odds -> probabilidad
        float p = 1.0f - (1.0f / (1.0f + std::exp(lo)));
        if (p > OCCUPIED_THRESH)    grid_data_[i] = 0;
        else if (p < FREE_THRESH)   grid_data_[i] = -1;
        else    grid_data_[i] = 100;
    }
}

void OccupancyGridBuilder::publish_map() {
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

bool OccupancyGridBuilder::update_laser_tf(const rclcpp::Time &required_time)
{ 
  try
  {
    // lookupTransform da la transformación que convierte puntos expresados en laser_frame al frame map_frame, válida en required_time
    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer.lookupTransform(
        map_frame, laser_frame, required_time, tf2::durationFromSec(0.1));
    tf2::fromMsg(transformStamped.transform, transform_laser_world);
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "No se pudo transformar %s a %s: %s",
                robot_frame.c_str(), laser_frame.c_str(), ex.what());
    return false;
  }
}

// Cuando necesite devuelvo directo gx y gy (Bresenham)
inline bool OccupancyGridBuilder::worldToIndex(double wx, double wy, int &index) const{
    int gx = static_cast<int>((wx - origin_x_) / resolution_);
    int gy = static_cast<int>((wy - origin_y_) / resolution_);

    if (gx < 0 || gx >= static_cast<int>(width_cells_) ||
        gy < 0 || gy >= static_cast<int>(height_cells_))
        return false;

    index = gy * width_cells_ + gx;
    return true;
}
