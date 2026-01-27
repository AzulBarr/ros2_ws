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

    this->declare_parameter("robot_frame", std::string("base_link"));
    //this->declare_parameter("publish_robot_frame", std::string("base_link"));
    this->declare_parameter("laser_frame", std::string("laser"));

    this->get_parameter("robot_frame", robot_frame);
    //this->get_parameter("publish_robot_frame", publish_robot_frame);
    this->get_parameter("laser_frame", laser_frame);
    

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
    grid_data_.assign(width_cells_ * height_cells_, -1); // equivale a: grid_data_.clear(); grid_data_.resize(width * height); for (...) grid_data_[i] = -1;

    map_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Mapa inicializado: %u x %u celdas", width_cells_, height_cells_);
}

void OccupancyGridBuilder::pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    current_pose_ = *msg;
    pose_received_ = true;
}

void OccupancyGridBuilder::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!map_initialized_ || !pose_received_) return;

    if (!update_laser_tf(msg->header.stamp))
    {
        RCLCPP_WARN(this->get_logger(), "%s -> %s transform not yet received", laser_frame.c_str(), robot_frame.c_str());
        return;
    }

    /* Convertir range,bearing a puntos cartesianos x,y,0.
    * Descartando aquellas mediciones por fuera de los rangos validos */

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
            p = laser_transform * p;
            update_cell_log_odds(p.x(), p.y(), l_occ_); // Sumamos probabilidad de ocupado
        } else {
            // CASO 2: NO DETECTÓ NADA (dist == -1)
            // Se marca espacio libre a lo largo del rayo.
            //float fx = (range_max * 0.5f) * cos(angle_min + angle_increment * i); // Ejemplo: punto medio
            //float fy = (range_max * 0.5f) * sin(angle_min + angle_increment * i);
            //tf2::Vector3 p(fx, fy, 0);
            //p = laser_transform * p;
            //update_cell_log_odds(p.x(), p.y(), l_free_); // Restamos probabilidad (espacio libre)
        }
        // AHORA HAY QUE CONVERTIRLO AL MARCO DE REFERENCIA DEL MAPA (WORLD)?
    }
    // 3. Convertir Log-Odds a OccupancyGrid (0-100) y publicar
    update_occupancy_grid();
    publish_map();
/*
    // 1. Obtener posición y rotación (Yaw) del robot
    float rx = current_pose_.position.x;
    float ry = current_pose_.position.y;
    float yaw = quaternion_to_yaw(current_pose_.orientation);

    // 2. Procesar cada rayo del láser
    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float dist = msg->ranges[i];
        
        if (dist >= msg->range_min && dist <= msg->range_max) {
            // CASO 1: DETECTÓ OBSTÁCULO
            float ox = rx + dist * std::cos(yaw + angle);
            float oy = ry + dist * std::sin(yaw + angle);
            update_cell_log_odds(ox, oy, l_occ_); // Sumamos probabilidad de ocupado

            // (Opcional) Limpiar el camino antes del obstáculo
            // limpiar_camino_libre(rx, ry, ox, oy); 
        } else {
            // CASO 2: NO DETECTÓ NADA (dist == -1)
            // Aquí es donde "limpias" el mapa. 
            // No marcas un obstáculo a los 8m, sino que marcas espacio libre 
            // en algún punto o a lo largo del rayo.
            float fx = rx + (dist * 0.5f) * std::cos(yaw + angle); // Ejemplo: punto medio
            float fy = ry + (dist * 0.5f) * std::sin(yaw + angle);
            update_cell_log_odds(fx, fy, l_free_); // Restamos probabilidad (espacio libre)
        }
            // OPCIONAL: Raycasting para marcar espacio libre entre robot y objeto
            // Aquí podrías implementar un algoritmo de Bresenham
        angle += msg->angle_increment;
    }
    // 3. Convertir Log-Odds a OccupancyGrid (0-100) y publicar
    update_occupancy_grid();
    publish_map();
*/
}

// ENTENDER Y MODIFICAR SI ES NECESARIO
void OccupancyGridBuilder::update_cell_log_odds(float wx, float wy, float l_update) {
    // Convertir coordenadas de mundo a índices de grilla
    int gx = static_cast<int>((wx - origin_x_) / resolution_);
    int gy = static_cast<int>((wy - origin_y_) / resolution_);

    if (gx >= 0 && gx < (int)width_cells_ && gy >= 0 && gy < (int)height_cells_) {
        int index = gy * width_cells_ + gx;
        
        // Sumar Log-Odds y aplicar límites (clamping)
        log_odds_grid_[index] += l_update;
        if (log_odds_grid_[index] > 5.0f) log_odds_grid_[index] = 5.0f;
        if (log_odds_grid_[index] < -5.0f) log_odds_grid_[index] = -5.0f;
    }
}

void OccupancyGridBuilder::update_occupancy_grid() {
    for (size_t i = 0; i < log_odds_grid_.size(); ++i) {
        float lo = log_odds_grid_[i];
        if (std::abs(lo) < 0.01f) {
            grid_data_[i] = -1; // Desconocido
        } else { // CHEQUEAR ESTO Y ACORDARSE DEL UMBRAL
            // Convertir de Log-Odds a probabilidad P = 1 - 1/(1 + exp(lo))
            float p = 1.0f - (1.0f / (1.0f + std::exp(lo)));
            grid_data_[i] = static_cast<int8_t>(p * 100.0f);
        }
    }
}

// FIJARSE QUE SEA IGUAL AL DE LA MATERIA
float OccupancyGridBuilder::quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
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
    // lookupTransform da la transformación que convierte puntos expresados en laser_frame al frame robot_frame, válida en required_time
    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer.lookupTransform(
        robot_frame, laser_frame, required_time, tf2::durationFromSec(0.1));
    tf2::fromMsg(transformStamped.transform, laser_transform);
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "No se pudo transformar %s a %s: %s",
                robot_frame.c_str(), laser_frame.c_str(), ex.what());
    return false;
  }
}