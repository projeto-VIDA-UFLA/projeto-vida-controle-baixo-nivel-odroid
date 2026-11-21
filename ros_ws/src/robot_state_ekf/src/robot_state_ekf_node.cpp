#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "vida_interfaces/msg/encoders.hpp"
#include "vida_interfaces/msg/steering.hpp"
#include "vida_interfaces/msg/gnss_status.hpp"
#include "vida_interfaces/msg/ekf_status.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

// Estimador de estado do robo (x, y, theta, phi, v, b) via Filtro de Kalman Estendido, convertido do
// RobotStateComponent (Pacpus/Heudiasyc) para ROS2. Onde o original lia GPS/IMU/CAN/odometria
// visual de memoria compartilhada a cada iteracao de uma thread com polling, este no usa
// callbacks de topico + um timer periodico, mas o nucleo do algoritmo (modelo cinematico Ackermann,
// predicao com jacobiano analitico, correcao na forma de Joseph) e o mesmo.
//
// Diferencas em relacao ao original:
//  - Sem VISO (nao ha odometria visual no novo conjunto de sensores): removida a correcao K3/C3.
//  - GPS: sem estacao Novatel/Lambert93; latitude/longitude sao convertidas para um plano local
//    ENU (leste/norte) por aproximacao equiretangular em torno do primeiro fix, com a variancia de
//    medida vinda de position_covariance quando disponivel (fallback para parametro fixo).
//  - CAN (angulo de esterçamento + velocidade): agora vem de dois topicos (Steering + Encoders,
//    com 3 encoders). O veiculo e' tracao traseira, entao o encoder traseiro fica sujeito a
//    derrapagem (patinagem sob torque de tracao/frenagem); a media dos encoders dianteiros
//    (rodas passivas, apenas esterçadas) e' usada como v por padrao - o pequeno erro sistematico
//    de raio ao esterçar e' bem menor que o erro de escorregamento na roda tracionada.
//  - IMU: o codigo original lia os frames de INS (raw IMU/INSPVAA/INSCOV) mas nunca os usava nas
//    correcoes do EKF (o switch em run() nao tratava esses casos). Aqui o giroscopio (Z) e
//    efetivamente usado para integrar theta, pois e tipicamente mais confiavel que o modelo
//    cinematico puro sob deslizamento das rodas; cai de volta para o modelo cinematico
//    (v*sin(phi)/L) se a IMU ficar sem dados por mais que `imu_timeout_sec`.
//  - Bias do giroscopio (b, 6o estado): o giroscopio mede w_real + b. Integrar sem descontar b
//    faz theta derivar sem limite (teste de 12/10/2026: -0,22 graus/s com o veiculo parado, e o
//    GPS so corrige theta indiretamente, quando o veiculo anda). Aqui theta_dot = w_medido - b,
//    com b modelado como passeio aleatorio (gyro_bias_random_walk). Com o veiculo parado
//    (encoders) o giro real e zero: uma pseudo-medida "giro zero" (z = w_medido, h(X) = b)
//    atualiza b e theta deixa de ser integrado. Antes da inicializacao, a media do giroscopio
//    parado da o valor inicial de b.
//  - Reset (servico ~/reset, usado ao entrar no modo automatico): zera o estado e refaz a
//    inicializacao (origem com gps_reset_init_samples fixes, bias parado, heading pelo GPS).
//  - Status (robot_state/status): fase da inicializacao, origem do heading e origem lat/lon do
//    frame local, publicados sempre (inclusive antes de inicializar).
//  - GNSS: com o GnssStatus, a incerteza do fix e inflada em RTK FLOAT/sem RTK (o receptor
//    informa hAcc otimista nesses estados) e o fix e ignorado sem solucao. O heading inicial pelo
//    GPS exige RTK FIXED (heading_init_require_fixed).
//  - Heading inicial: o original assumia theta=0 (ou vindo de config) na primeira aquisicao. Como
//    nao ha bussola/heading absoluto disponivel, apos fixar a origem GNSS o no aguarda o veiculo
//    se deslocar `heading_init_min_distance_m` para estimar o heading inicial pelo rumo do GPS
//    (atan2 do deslocamento entre dois fixes) - muito mais proximo da orientacao real do que um
//    valor fixo de parametro. Cai de volta para `initial_heading_deg` se o veiculo nao andar o
//    suficiente dentro de `heading_init_timeout_sec` (ex: ligou parado).
class RobotStateEkfNode : public rclcpp::Node
{
    // Estado: x, y, theta, phi, v, b (bias do giroscopio Z)
    static constexpr int kStateSize = 6;
    static constexpr int kBias = 5;
    using StateVector = Eigen::Matrix<double, kStateSize, 1>;
    using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;

public:
    RobotStateEkfNode() : Node("robot_state_ekf_node") {}

    void init()
    {
        declareParameters();

        Q_ = StateMatrix::Zero();
        Q_(0, 0) = model_noise_x_;
        Q_(1, 1) = model_noise_y_;
        Q_(2, 2) = model_noise_theta_;
        Q_(3, 3) = model_noise_phi_;
        Q_(4, 4) = model_noise_v_;

        R_can_ = Eigen::Matrix2d::Zero();
        R_can_(0, 0) = steering_sigma_rad_ * steering_sigma_rad_;
        R_can_(1, 1) = velocity_sigma_mps_ * velocity_sigma_mps_;

        init_samples_target_ = gps_init_samples_;
        resetFilter();

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
        steering_pub_ = create_publisher<std_msgs::msg::Float64>(steering_angle_topic_, 10);
        gyro_bias_pub_ = create_publisher<std_msgs::msg::Float64>(gyro_bias_topic_, 10);
        status_pub_ = create_publisher<vida_interfaces::msg::EkfStatus>(status_topic_, 10);
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RobotStateEkfNode::gpsCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RobotStateEkfNode::imuCallback, this, std::placeholders::_1));

        encoders_sub_ = create_subscription<vida_interfaces::msg::Encoders>(
            encoders_topic_, 10,
            std::bind(&RobotStateEkfNode::encodersCallback, this, std::placeholders::_1));

        steering_sub_ = create_subscription<vida_interfaces::msg::Steering>(
            steering_topic_, 10,
            std::bind(&RobotStateEkfNode::steeringCallback, this, std::placeholders::_1));

        gnss_status_sub_ = create_subscription<vida_interfaces::msg::GnssStatus>(
            gps_status_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RobotStateEkfNode::gnssStatusCallback, this, std::placeholders::_1));

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/reset",
            std::bind(&RobotStateEkfNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

        last_predict_time_ = now();
        ekf_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / ekf_rate_hz_),
            std::bind(&RobotStateEkfNode::ekfTimerCallback, this));

        publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_hz_),
            std::bind(&RobotStateEkfNode::publishTimerCallback, this));

        RCLCPP_INFO(get_logger(), "robot_state_ekf_node iniciado (wheelbase=%.3f m, ekf_rate=%.1f Hz, publish_rate=%.1f Hz)",
                    wheelbase_, ekf_rate_hz_, publish_rate_hz_);
    }

private:
    // ------------------------------------------------------------------ Parametros
    void declareParameters()
    {
        wheelbase_ = declare_parameter<double>("wheelbase", 1.125);
        phi_max_deg_ = declare_parameter<double>("phi_max_deg", 35.0);
        phi_max_ = phi_max_deg_ * M_PI / 180.0;

        ekf_rate_hz_ = declare_parameter<double>("ekf_rate_hz", 50.0);
        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

        use_gps_ = declare_parameter<bool>("use_gps", true);
        gps_init_samples_ = declare_parameter<int>("gps_init_samples", 10);
        gps_reset_init_samples_ = declare_parameter<int>("gps_reset_init_samples", 3);
        gps_float_sigma_m_ = declare_parameter<double>("gps_float_sigma_m", 0.3);
        gps_no_rtk_sigma_m_ = declare_parameter<double>("gps_no_rtk_sigma_m", 1.0);
        heading_init_require_fixed_ = declare_parameter<bool>("heading_init_require_fixed", true);

        use_imu_heading_ = declare_parameter<bool>("use_imu_heading", true);
        imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.5);
        imu_yaw_rate_sigma_ = declare_parameter<double>("imu_yaw_rate_sigma", 0.02);

        estimate_gyro_bias_ = declare_parameter<bool>("estimate_gyro_bias", true);
        gyro_bias_initial_sigma_ = declare_parameter<double>("gyro_bias_initial_sigma", 0.01);
        gyro_bias_calibrated_sigma_ = declare_parameter<double>("gyro_bias_calibrated_sigma", 0.0005);
        gyro_bias_random_walk_ = declare_parameter<double>("gyro_bias_random_walk", 2.0e-4);
        stationary_speed_threshold_ = declare_parameter<double>("stationary_speed_threshold_mps", 0.02);
        stationary_min_time_sec_ = declare_parameter<double>("stationary_min_time_sec", 0.5);
        encoders_timeout_sec_ = declare_parameter<double>("encoders_timeout_sec", 0.5);

        pose_from_file_ = declare_parameter<bool>("pose_from_file", false);
        initial_latitude_deg_ = declare_parameter<double>("initial_latitude_deg", 0.0);
        initial_longitude_deg_ = declare_parameter<double>("initial_longitude_deg", 0.0);
        initial_heading_deg_ = declare_parameter<double>("initial_heading_deg", 0.0);

        estimate_heading_from_gps_ = declare_parameter<bool>("estimate_heading_from_gps", true);
        heading_init_min_distance_m_ = declare_parameter<double>("heading_init_min_distance_m", 2.0);
        heading_init_timeout_sec_ = declare_parameter<double>("heading_init_timeout_sec", 20.0);

        velocity_source_ = declare_parameter<std::string>("velocity_source", "front_average");

        model_noise_x_ = declare_parameter<double>("model_noise_x", 0.01);
        model_noise_y_ = declare_parameter<double>("model_noise_y", 0.01);
        model_noise_theta_ = declare_parameter<double>("model_noise_theta", 0.001);
        model_noise_phi_ = declare_parameter<double>("model_noise_phi", 0.001);
        model_noise_v_ = declare_parameter<double>("model_noise_v", 0.01);

        steering_sigma_rad_ = declare_parameter<double>("steering_sigma_rad", 0.02);
        velocity_sigma_mps_ = declare_parameter<double>("velocity_sigma_mps", 0.05);
        gps_default_sigma_m_ = declare_parameter<double>("gps_default_sigma_m", 2.0);

        publish_tf_ = declare_parameter<bool>("publish_tf", true);
        odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
        base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");

        gps_topic_ = declare_parameter<std::string>("gps_topic", "/vida/sensors/gps/fix");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/vida/sensors/imu/data_raw");
        encoders_topic_ = declare_parameter<std::string>("encoders_topic", "/vida/sensors/encoders");
        steering_topic_ = declare_parameter<std::string>("steering_topic", "/vida/sensors/steering");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "robot_state/odom");
        steering_angle_topic_ = declare_parameter<std::string>("steering_angle_topic", "robot_state/steering_angle");
        gyro_bias_topic_ = declare_parameter<std::string>("gyro_bias_topic", "robot_state/gyro_bias");
        status_topic_ = declare_parameter<std::string>("status_topic", "robot_state/status");
        gps_status_topic_ = declare_parameter<std::string>("gps_status_topic", "/vida/sensors/gps/status");
    }

    // ------------------------------------------------------------------ Conversao GNSS -> ENU local
    // Substitui o transf_lamb_93 (Lambert93, especifico da France) do codigo original por uma
    // aproximacao equiretangular em torno do primeiro fix - adequada para a escala de operacao de
    // um robo terrestre (poucos km), independente de projecao cartografica regional.
    void setOriginFromParams()
    {
        origin_lat_ = initial_latitude_deg_;
        origin_lon_ = initial_longitude_deg_;
        computeLocalRadii(origin_lat_);
        has_origin_ = true;
        ++origin_id_;
    }

    void computeLocalRadii(double lat_deg)
    {
        constexpr double kSemiMajorAxis = 6378137.0;      // WGS84 a
        constexpr double kEccentricitySq = 6.69437999014e-3; // WGS84 e^2
        const double lat_rad = lat_deg * M_PI / 180.0;
        const double sin_lat = std::sin(lat_rad);
        const double denom = 1.0 - kEccentricitySq * sin_lat * sin_lat;
        const double N = kSemiMajorAxis / std::sqrt(denom);              // raio de curvatura primeiro vertical
        const double M = kSemiMajorAxis * (1.0 - kEccentricitySq) / std::pow(denom, 1.5); // raio meridiano
        meters_per_rad_lat_ = M;
        meters_per_rad_lon_ = N * std::cos(lat_rad);
    }

    void latLonToLocalXY(double lat_deg, double lon_deg, double & x, double & y) const
    {
        const double dlat_rad = (lat_deg - origin_lat_) * M_PI / 180.0;
        const double dlon_rad = (lon_deg - origin_lon_) * M_PI / 180.0;
        y = dlat_rad * meters_per_rad_lat_;  // norte
        x = dlon_rad * meters_per_rad_lon_;  // leste
    }

    // ------------------------------------------------------------------ Callbacks dos sensores
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!has_origin_) {
            gps_init_lat_sum_ += msg->latitude;
            gps_init_lon_sum_ += msg->longitude;
            ++gps_init_count_;

            if (gps_init_count_ >= init_samples_target_) {
                origin_lat_ = gps_init_lat_sum_ / gps_init_count_;
                origin_lon_ = gps_init_lon_sum_ / gps_init_count_;
                computeLocalRadii(origin_lat_);
                has_origin_ = true;
                ++origin_id_;

                RCLCPP_INFO(get_logger(), "Origem GNSS local fixada apos %d amostras (lat=%.7f, lon=%.7f).",
                            gps_init_count_, origin_lat_, origin_lon_);

                if (estimate_heading_from_gps_) {
                    heading_ref_time_ = now();
                } else {
                    X_(0, 0) = 0.0;
                    X_(1, 0) = 0.0;
                    X_(2, 0) = initial_heading_deg_ * M_PI / 180.0;
                    heading_source_ = vida_interfaces::msg::EkfStatus::HEADING_PARAMETER;
                    initialized_ = true;
                }
            }
            return;
        }

        if (!initialized_) {
            // Origem fixada, aguardando deslocamento para estimar o heading inicial pelo rumo do
            // GPS (mais confiavel que assumir um valor fixo, ja que a orientacao real na largada
            // costuma nao bater com initial_heading_deg).
            double x, y;
            latLonToLocalXY(msg->latitude, msg->longitude, x, y);

            // Rumo de dois fixes so e confiavel com precisao de centimetros (RTK FIXED).
            const bool fix_good_for_heading = !heading_init_require_fixed_ || !gnss_status_received_ ||
                gps_rtk_status_ == vida_interfaces::msg::GnssStatus::RTK_FIXED;
            if (!heading_ref_set_) {
                if (fix_good_for_heading) {
                    heading_ref_xy_ = Eigen::Vector2d(x, y);
                    heading_ref_set_ = true;
                }
                return;
            }
            if (!fix_good_for_heading) {
                // Perdeu o FIXED no meio: recomeca a referencia.
                heading_ref_set_ = false;
                return;
            }

            const Eigen::Vector2d current_xy(x, y);
            const Eigen::Vector2d delta = current_xy - heading_ref_xy_;
            const double distance = delta.norm();

            if (distance >= heading_init_min_distance_m_) {
                X_(0, 0) = x;
                X_(1, 0) = y;
                X_(2, 0) = std::atan2(delta.y(), delta.x());
                heading_source_ = vida_interfaces::msg::EkfStatus::HEADING_GPS;
                initialized_ = true;
                RCLCPP_INFO(get_logger(),
                            "Heading inicial estimado via GPS: %.1f graus (deslocamento de %.2f m).",
                            X_(2, 0) * 180.0 / M_PI, distance);
            } else if ((now() - heading_ref_time_).seconds() > heading_init_timeout_sec_) {
                X_(0, 0) = x;
                X_(1, 0) = y;
                X_(2, 0) = initial_heading_deg_ * M_PI / 180.0;
                heading_source_ = vida_interfaces::msg::EkfStatus::HEADING_PARAMETER;
                initialized_ = true;
                RCLCPP_WARN(get_logger(),
                            "Veiculo nao se deslocou o suficiente (%.2f m em %.0f s) para estimar o heading via GPS; "
                            "usando initial_heading_deg=%.1f como fallback.",
                            distance, heading_init_timeout_sec_, initial_heading_deg_);
            }
            return;
        }

        if (!use_gps_) {
            return;
        }

        // Sem solucao valida o fix nao entra no filtro.
        if (gnss_status_received_ && !gnss_fix_ok_) {
            return;
        }

        double x, y;
        latLonToLocalXY(msg->latitude, msg->longitude, x, y);
        last_gps_xy_ = Eigen::Vector2d(x, y);

        if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
            // position_covariance e' dado no referencial ENU local: [0]=leste-leste, [4]=norte-norte
            last_gps_R_ = Eigen::Matrix2d::Zero();
            last_gps_R_(0, 0) = msg->position_covariance[0];
            last_gps_R_(1, 1) = msg->position_covariance[4];
            last_gps_R_(0, 1) = last_gps_R_(1, 0) = msg->position_covariance[1];
        } else {
            last_gps_R_ = Eigen::Matrix2d::Identity() * (gps_default_sigma_m_ * gps_default_sigma_m_);
        }

        // O receptor informa hAcc de centimetros mesmo em FLOAT: sem FIXED a variancia tem piso.
        if (gnss_status_received_ && gps_rtk_status_ != vida_interfaces::msg::GnssStatus::RTK_FIXED) {
            const double sigma = gps_rtk_status_ == vida_interfaces::msg::GnssStatus::RTK_FLOAT
                ? gps_float_sigma_m_ : gps_no_rtk_sigma_m_;
            last_gps_R_(0, 0) = std::max(last_gps_R_(0, 0), sigma * sigma);
            last_gps_R_(1, 1) = std::max(last_gps_R_(1, 1), sigma * sigma);
        }

        new_gps_data_ = true;
    }

    void gnssStatusCallback(const vida_interfaces::msg::GnssStatus::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        gnss_status_received_ = true;
        gnss_fix_ok_ = msg->gnss_fix_ok;
        gps_rtk_status_ = msg->rtk_status;
    }

    void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        init_samples_target_ = gps_reset_init_samples_;
        resetFilter();
        response->success = true;
        response->message = "EKF reiniciado: aguardando origem, bias parado e heading pelo GPS";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }

    // Estado inicial: zera o filtro e refaz a inicializacao (origem, heading e bias).
    void resetFilter()
    {
        X_ = StateVector::Zero();
        P_ = StateMatrix::Identity() * 10.0;
        P_(kBias, kBias) = gyro_bias_initial_sigma_ * gyro_bias_initial_sigma_;

        initialized_ = false;
        has_origin_ = false;
        gps_init_lat_sum_ = gps_init_lon_sum_ = 0.0;
        gps_init_count_ = 0;
        heading_ref_set_ = false;
        heading_source_ = vida_interfaces::msg::EkfStatus::HEADING_NONE;
        new_gps_data_ = false;

        initial_bias_sum_ = initial_bias_sum_sq_ = 0.0;
        initial_bias_count_ = 0;
        initial_bias_applied_ = false;
        stationary_since_set_ = false;
        last_predict_time_ = now();

        if (pose_from_file_) {
            setOriginFromParams();
            X_(2, 0) = initial_heading_deg_ * M_PI / 180.0;
            heading_source_ = vida_interfaces::msg::EkfStatus::HEADING_PARAMETER;
            initialized_ = true;
            RCLCPP_INFO(get_logger(), "Estado inicial definido via parametros (pose_from_file=true).");
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_yaw_rate_ = msg->angular_velocity.z;
        last_imu_stamp_ = now();

        // Covariancia -1 (sem dado) ou 0 (o driver do ZED-F9R publica zeros) e tratada como
        // desconhecida: com variancia zero o filtro tomaria o giroscopio como perfeito (theta
        // sem incerteza de processo e o bias pulando para cada amostra ruidosa).
        if (msg->angular_velocity_covariance[0] >= 0.0 && msg->angular_velocity_covariance[8] > 0.0) {
            // indice [2][2] (zz) de uma matriz 3x3 linearizada -> indice 8
            last_yaw_rate_var_ = msg->angular_velocity_covariance[8];
        } else {
            last_yaw_rate_var_ = imu_yaw_rate_sigma_ * imu_yaw_rate_sigma_;
        }
    }

    void encodersCallback(const vida_interfaces::msg::Encoders::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (velocity_source_ == "front_left") {
            last_velocity_mps_ = msg->left.filtered_velocity_m_s;
        } else if (velocity_source_ == "front_right") {
            last_velocity_mps_ = msg->right.filtered_velocity_m_s;
        } else if (velocity_source_ == "rear") {
            last_velocity_mps_ = msg->rear.filtered_velocity_m_s;
        } else {
            last_velocity_mps_ = 0.5 * (msg->left.filtered_velocity_m_s + msg->right.filtered_velocity_m_s);
        }

        last_encoders_stamp_ = now();
        has_can_data_ = true;
    }

    void steeringCallback(const vida_interfaces::msg::Steering::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_steering_angle_rad_ = msg->filtered_angle_deg * M_PI / 180.0;
        has_can_data_ = true;
    }

    // ------------------------------------------------------------------ Predicao (modelo Ackermann)
    // Equivalente a RobotKinematicModel + EKF_Prediction2/Direct_Prediction do codigo original: phi
    // e v sao tratados como entradas exogenas (substituidas pela leitura mais recente do
    // esterçamento/encoders a cada ciclo), nao como estados livremente integrados.
    void predict(double dt, bool stationary)
    {
        X_(3, 0) = last_steering_angle_rad_;
        X_(4, 0) = last_velocity_mps_;

        if (std::abs(X_(3, 0)) > phi_max_) {
            X_(3, 0) = std::copysign(phi_max_, X_(3, 0));
        }

        const double theta = X_(2, 0);
        const double phi = X_(3, 0);
        const double v = X_(4, 0);

        const bool imu_fresh = use_imu_heading_ &&
            (now() - last_imu_stamp_).seconds() < imu_timeout_sec_;

        const double bias = X_(kBias, 0);

        double theta_dot;
        double theta_process_noise;
        if (stationary && estimate_gyro_bias_) {
            // Parado o veiculo nao gira: theta nao e integrado (o giroscopio so mede o bias).
            theta_dot = 0.0;
            theta_process_noise = 0.0;
        } else if (imu_fresh) {
            theta_dot = estimate_gyro_bias_ ? last_yaw_rate_ - bias : last_yaw_rate_;
            theta_process_noise = last_yaw_rate_var_ * dt * dt;
        } else {
            theta_dot = v * std::sin(phi) / wheelbase_;
            theta_process_noise = model_noise_theta_;
        }

        X_(0, 0) = X_(0, 0) + v * std::cos(theta) * std::cos(phi) * dt;
        X_(1, 0) = X_(1, 0) + v * std::sin(theta) * std::cos(phi) * dt;
        X_(2, 0) = theta + theta_dot * dt;

        if (X_(2, 0) > M_PI) {
            X_(2, 0) -= 2.0 * M_PI;
        } else if (X_(2, 0) < -M_PI) {
            X_(2, 0) += 2.0 * M_PI;
        }

        StateMatrix A = StateMatrix::Identity();
        A(0, 2) = -v * std::sin(theta) * std::cos(phi) * dt;
        A(0, 3) = -v * std::cos(theta) * std::sin(phi) * dt;
        A(0, 4) = std::cos(theta) * std::cos(phi) * dt;
        A(1, 2) = v * std::cos(theta) * std::cos(phi) * dt;
        A(1, 3) = -v * std::sin(theta) * std::sin(phi) * dt;
        A(1, 4) = std::sin(theta) * std::cos(phi) * dt;

        if (!(stationary && estimate_gyro_bias_) && !imu_fresh) {
            A(2, 3) = v * (std::cos(phi) / wheelbase_) * dt;
            A(2, 4) = (std::sin(phi) / wheelbase_) * dt;
        }
        // com IMU fresca, theta_dot independe de phi/v: A(2,3)=A(2,4)=0 (ja e' o caso na identidade)
        if (!stationary && imu_fresh && estimate_gyro_bias_) {
            A(2, kBias) = -dt;  // d(theta)/d(b)
        }

        Q_(2, 2) = theta_process_noise;
        // Bias como passeio aleatorio: variancia cresce gyro_bias_random_walk^2 por segundo.
        Q_(kBias, kBias) = estimate_gyro_bias_
            ? gyro_bias_random_walk_ * gyro_bias_random_walk_ * dt
            : 0.0;
        P_ = A * P_ * A.transpose() + Q_;
    }

    // Correcao generica na forma de Joseph (estabilizada), equivalente a
    // EKF_Extimation_GPS/GPSL/CAN do original (que duplicavam a mesma formula 3x).
    template <int M>
    void applyCorrection(const Eigen::Matrix<double, M, kStateSize> & H,
                         const Eigen::Matrix<double, M, M> & R,
                         const Eigen::Matrix<double, M, 1> & measurement)
    {
        const Eigen::Matrix<double, M, 1> prediction = H * X_;
        const Eigen::Matrix<double, M, 1> innovation = measurement - prediction;
        const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
        const Eigen::Matrix<double, kStateSize, M> K = P_ * H.transpose() * S.inverse();

        X_ = X_ + K * innovation;

        const StateMatrix I_KH = StateMatrix::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    }

    // Veiculo parado: encoders recentes abaixo do limiar por stationary_min_time_sec.
    bool updateStationary(const rclcpp::Time & t_now)
    {
        const bool encoders_fresh = has_can_data_ &&
            (t_now - last_encoders_stamp_).seconds() < encoders_timeout_sec_;
        if (!encoders_fresh || std::abs(last_velocity_mps_) >= stationary_speed_threshold_) {
            stationary_since_set_ = false;
            return false;
        }
        if (!stationary_since_set_) {
            stationary_since_ = t_now;
            stationary_since_set_ = true;
        }
        return (t_now - stationary_since_).seconds() >= stationary_min_time_sec_;
    }

    void ekfTimerCallback()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        const rclcpp::Time t_now = now();
        const bool stationary = updateStationary(t_now);
        last_stationary_ = stationary;
        const bool imu_fresh = use_imu_heading_ &&
            (t_now - last_imu_stamp_).seconds() < imu_timeout_sec_;

        if (!initialized_) {
            // Mantem last_predict_time_ em dia enquanto aguarda a inicializacao (origem GNSS +
            // heading via GPS): sem isso, o primeiro predict() real usaria um dt gigante (todo o
            // tempo desde o start do no), estourando theta com um unico passo de integracao do
            // giroscopio (ou x,y no modelo cinematico) assim que initialized_ vira true.
            last_predict_time_ = t_now;

            // Enquanto espera, a media do giroscopio parado da o valor inicial do bias.
            if (estimate_gyro_bias_ && stationary && imu_fresh) {
                initial_bias_sum_ += last_yaw_rate_;
                initial_bias_sum_sq_ += last_yaw_rate_ * last_yaw_rate_;
                ++initial_bias_count_;
            }
            return;
        }

        if (!initial_bias_applied_) {
            initial_bias_applied_ = true;
            if (estimate_gyro_bias_ && initial_bias_count_ >= kMinInitialBiasSamples) {
                const double n = static_cast<double>(initial_bias_count_);
                const double mean = initial_bias_sum_ / n;
                const double variance = std::max(0.0, initial_bias_sum_sq_ / n - mean * mean);
                // Incerteza da media (desvio/sqrt(n)), com piso: as amostras nao sao independentes.
                const double sigma = std::max(std::sqrt(variance / n), gyro_bias_calibrated_sigma_);
                X_(kBias, 0) = mean;
                P_(kBias, kBias) = sigma * sigma;
                RCLCPP_INFO(get_logger(), "Bias inicial do giroscopio: %.5f rad/s (%.3f graus/s, sigma %.5f, %d amostras parado).",
                            mean, mean * 180.0 / M_PI, sigma, initial_bias_count_);
            }
        }

        const double dt = (t_now - last_predict_time_).seconds();
        last_predict_time_ = t_now;

        if (dt <= 0.0) {
            return;
        }

        predict(dt, stationary);

        // Giro zero parado: o giroscopio mede so o bias (z = w_medido, h(X) = b).
        if (estimate_gyro_bias_ && stationary && imu_fresh) {
            Eigen::Matrix<double, 1, kStateSize> H_zero_rate = Eigen::Matrix<double, 1, kStateSize>::Zero();
            H_zero_rate(0, kBias) = 1.0;
            applyCorrection<1>(H_zero_rate,
                               Eigen::Matrix<double, 1, 1>(last_yaw_rate_var_),
                               Eigen::Matrix<double, 1, 1>(last_yaw_rate_));
        }

        if (has_can_data_) {
            static const Eigen::Matrix<double, 2, kStateSize> H_can =
                (Eigen::Matrix<double, 2, kStateSize>() <<
                    0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1, 0).finished();
            applyCorrection<2>(H_can, R_can_, Eigen::Vector2d(last_steering_angle_rad_, last_velocity_mps_));
        }

        if (new_gps_data_) {
            static const Eigen::Matrix<double, 2, kStateSize> H_gps =
                (Eigen::Matrix<double, 2, kStateSize>() <<
                    1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0).finished();
            applyCorrection<2>(H_gps, last_gps_R_, last_gps_xy_);
            new_gps_data_ = false;
        }
    }

    // ------------------------------------------------------------------ Publicacao
    void publishTimerCallback()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        const rclcpp::Time stamp = now();
        publishStatus(stamp);

        if (!initialized_) {
            return;
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, X_(2, 0));

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = base_frame_id_;

        odom.pose.pose.position.x = X_(0, 0);
        odom.pose.pose.position.y = X_(1, 0);
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(q);

        std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
        odom.pose.covariance[0] = P_(0, 0);
        odom.pose.covariance[1] = P_(0, 1);
        odom.pose.covariance[6] = P_(1, 0);
        odom.pose.covariance[7] = P_(1, 1);
        odom.pose.covariance[35] = P_(2, 2);

        const double yaw_rate = X_(4, 0) * std::sin(X_(3, 0)) / wheelbase_;
        odom.twist.twist.linear.x = X_(4, 0);
        odom.twist.twist.angular.z = yaw_rate;
        std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
        odom.twist.covariance[0] = P_(4, 4);

        odom_pub_->publish(odom);

        std_msgs::msg::Float64 steering_msg;
        steering_msg.data = X_(3, 0);
        steering_pub_->publish(steering_msg);

        std_msgs::msg::Float64 bias_msg;
        bias_msg.data = X_(kBias, 0);
        gyro_bias_pub_->publish(bias_msg);

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = odom_frame_id_;
            tf_msg.child_frame_id = base_frame_id_;
            tf_msg.transform.translation.x = X_(0, 0);
            tf_msg.transform.translation.y = X_(1, 0);
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    void publishStatus(const rclcpp::Time & stamp)
    {
        vida_interfaces::msg::EkfStatus status;
        status.timestamp = stamp;
        status.phase = initialized_ ? vida_interfaces::msg::EkfStatus::PHASE_RUNNING
                     : has_origin_  ? vida_interfaces::msg::EkfStatus::PHASE_WAITING_HEADING
                                    : vida_interfaces::msg::EkfStatus::PHASE_WAITING_ORIGIN;
        status.heading_source = heading_source_;
        status.origin_valid = has_origin_;
        status.origin_latitude_deg = origin_lat_;
        status.origin_longitude_deg = origin_lon_;
        status.origin_id = origin_id_;
        status.stationary = last_stationary_;
        status.gyro_bias_rad_s = X_(kBias, 0);
        status.bias_calibration_samples = static_cast<uint32_t>(initial_bias_count_);
        status.gps_rtk_status = gnss_status_received_ ? gps_rtk_status_
                                                      : vida_interfaces::msg::GnssStatus::RTK_NONE;
        status_pub_->publish(status);
    }

    // ------------------------------------------------------------------ Parametros (membros)
    double wheelbase_;
    double phi_max_deg_;
    double phi_max_;
    double ekf_rate_hz_;
    double publish_rate_hz_;
    bool use_gps_;
    int gps_init_samples_;
    int gps_reset_init_samples_;
    int init_samples_target_ = 10;  // fixes da origem: boot ou reset
    double gps_float_sigma_m_;
    double gps_no_rtk_sigma_m_;
    bool heading_init_require_fixed_;
    bool use_imu_heading_;
    double imu_timeout_sec_;
    double imu_yaw_rate_sigma_;
    bool estimate_gyro_bias_;
    double gyro_bias_initial_sigma_;
    double gyro_bias_calibrated_sigma_;
    double gyro_bias_random_walk_;
    double stationary_speed_threshold_;
    double stationary_min_time_sec_;
    double encoders_timeout_sec_;
    bool pose_from_file_;
    double initial_latitude_deg_;
    double initial_longitude_deg_;
    double initial_heading_deg_;
    bool estimate_heading_from_gps_;
    double heading_init_min_distance_m_;
    double heading_init_timeout_sec_;
    std::string velocity_source_;
    double model_noise_x_, model_noise_y_, model_noise_theta_, model_noise_phi_, model_noise_v_;
    double steering_sigma_rad_;
    double velocity_sigma_mps_;
    double gps_default_sigma_m_;
    bool publish_tf_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string gps_topic_, imu_topic_, encoders_topic_, steering_topic_, odom_topic_, steering_angle_topic_;
    std::string gyro_bias_topic_;
    std::string status_topic_;
    std::string gps_status_topic_;

    // ------------------------------------------------------------------ Estado do EKF
    StateVector X_;   // (x, y, theta, phi, v, b)
    StateMatrix P_;
    StateMatrix Q_;
    Eigen::Matrix2d R_can_;

    bool initialized_ = false;
    bool has_origin_ = false;
    double origin_lat_ = 0.0, origin_lon_ = 0.0;
    double meters_per_rad_lat_ = 0.0, meters_per_rad_lon_ = 0.0;
    double gps_init_lat_sum_ = 0.0, gps_init_lon_sum_ = 0.0;
    int gps_init_count_ = 0;

    bool heading_ref_set_ = false;
    Eigen::Vector2d heading_ref_xy_ = Eigen::Vector2d::Zero();
    rclcpp::Time heading_ref_time_{0, 0, RCL_ROS_TIME};

    double last_steering_angle_rad_ = 0.0;
    double last_velocity_mps_ = 0.0;
    bool has_can_data_ = false;
    rclcpp::Time last_encoders_stamp_{0, 0, RCL_ROS_TIME};

    // Veiculo parado e bias inicial
    bool stationary_since_set_ = false;
    rclcpp::Time stationary_since_{0, 0, RCL_ROS_TIME};
    static constexpr int kMinInitialBiasSamples = 25;  // 0,5 s a 50 Hz
    double initial_bias_sum_ = 0.0;
    double initial_bias_sum_sq_ = 0.0;
    int initial_bias_count_ = 0;
    bool initial_bias_applied_ = false;
    bool last_stationary_ = false;

    // Origem e GNSS (status)
    uint32_t origin_id_ = 0;
    uint8_t heading_source_ = 0;  // EkfStatus::HEADING_*
    bool gnss_status_received_ = false;
    bool gnss_fix_ok_ = false;
    uint8_t gps_rtk_status_ = 0;  // GnssStatus::RTK_*

    double last_yaw_rate_ = 0.0;
    double last_yaw_rate_var_ = 0.0;
    rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};

    bool new_gps_data_ = false;
    Eigen::Vector2d last_gps_xy_ = Eigen::Vector2d::Zero();
    Eigen::Matrix2d last_gps_R_ = Eigen::Matrix2d::Identity();

    rclcpp::Time last_predict_time_;

    std::mutex data_mutex_;

    // ------------------------------------------------------------------ ROS I/O
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<vida_interfaces::msg::Encoders>::SharedPtr encoders_sub_;
    rclcpp::Subscription<vida_interfaces::msg::Steering>::SharedPtr steering_sub_;
    rclcpp::Subscription<vida_interfaces::msg::GnssStatus>::SharedPtr gnss_status_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Publisher<vida_interfaces::msg::EkfStatus>::SharedPtr status_pub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gyro_bias_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr ekf_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStateEkfNode>();
    try {
        node->init();
    } catch (const std::exception & e) {
        RCLCPP_FATAL(node->get_logger(), "Falha ao inicializar robot_state_ekf_node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
