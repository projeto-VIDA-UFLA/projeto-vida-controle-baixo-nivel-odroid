# robot_state_ekf

Estimador de estado do robô — `(x, y, theta, phi, v, b)` — via Filtro de Kalman Estendido, fundindo
GNSS, IMU, encoders e ângulo de esterçamento. Conversão para ROS2 do `RobotStateComponent`
(Pacpus/Heudiasyc UMR UTC/CNRS 6599) original.

## Como funciona

1. Assina `gps_topic` (`sensor_msgs/NavSatFix`), `imu_topic` (`sensor_msgs/Imu`),
   `encoders_topic` (`vida_interfaces/Encoders`) e `steering_topic` (`vida_interfaces/Steering`).
2. Fixa uma origem GNSS local (média de `gps_init_samples` fixes) e, em seguida, estima o heading
   inicial pelo rumo do GPS (deslocamento entre dois fixes) — veja `estimate_heading_from_gps` em
   `config/robot_state_ekf_params.yaml`.
3. Roda um timer de predição/correção a `ekf_rate_hz` (modelo cinemático Ackermann, phi/v vindos
   diretamente do esterçamento/encoders) e um timer de publicação a `publish_rate_hz`.
4. Publica:
   - `odom_topic` (`nav_msgs/Odometry`, default `robot_state/odom`) — pose (x, y, yaw) e twist
     (v, yaw rate), com covariância.
   - `steering_angle_topic` (`std_msgs/Float64`, default `robot_state/steering_angle`) — phi
     estimado, em radianos.
   - `gyro_bias_topic` (`std_msgs/Float64`, default `robot_state/gyro_bias`) — bias estimado do
     giroscópio Z, em rad/s.
   - `status_topic` (`vida_interfaces/EkfStatus`, default `robot_state/status`) — fase da
     inicialização, origem do heading (GPS ou parâmetro), origem lat/lon do frame local
     (`origin_id` muda a cada nova origem), bias e status RTK. Publicado sempre, inclusive antes
     de inicializar.
5. Oferece o serviço `~/reset` (`std_srvs/Trigger`), usado ao entrar no modo automático: zera o
   estado e refaz a inicialização (origem com `gps_reset_init_samples` fixes, bias parado, heading
   pelo GPS).
   - tf `odom_frame_id -> base_frame_id` (default `odom -> base_link`), se `publish_tf: true`.

Todos os parâmetros (tópicos, geometria do veículo, ruídos, fonte de velocidade dos encoders,
pose inicial) estão documentados em `config/robot_state_ekf_params.yaml`.

## GNSS e status RTK

Com o `GnssStatus` (`gps_status_topic`):

- **Sem solução** (`gnss_fix_ok = false`): o fix é ignorado.
- **RTK FLOAT / sem RTK:** a incerteza do fix tem piso (`gps_float_sigma_m`, `gps_no_rtk_sigma_m`),
  porque o receptor informa hAcc de centímetros nesses estados.
- **Heading inicial pelo GPS só em RTK FIXED** (`heading_init_require_fixed`): o rumo entre dois
  fixes a ~2 m só tem precisão com erro de centímetros.

Sem o tópico de status, o comportamento é o anterior.

Observação (rosbag de 12/10/2026): com o GPS a 1 Hz, a posição dá um degrau de até ~0,35 m a cada
fix, porque a navegação por encoders/giroscópio desvia entre fixes (escala dos encoders ~7%
acima do GPS). Calibrar a escala dos encoders e subir a taxa do GPS (o ZED-F9R suporta 5–10 Hz)
reduzem esses degraus.

## Bias do giroscópio

O giroscópio mede `ω_real + b`. O heading é integrado do giroscópio, então um bias não
compensado vira deriva sem limite. O GPS só mede posição e corrige o heading apenas
indiretamente, quando o veículo anda; parado, nada corrige.

O bias `b` é o 6º estado do filtro:

- **Predição:** `θ ← θ + (ω_medido − b)·dt`; `b` é um passeio aleatório (`gyro_bias_random_walk`).
- **Veículo parado** (média das rodas dianteiras abaixo de `stationary_speed_threshold_mps` por
  `stationary_min_time_sec`, com encoders recentes): o giro real é zero, então
  - uma pseudo-medida "giro zero" (`z = ω_medido`, `h(X) = b`) atualiza `b`;
  - θ não é integrado.
- **Antes da inicialização** (esperando o GPS), a média do giroscópio parado dá o valor inicial
  de `b`, com incerteza desvio/√n (piso `gyro_bias_calibrated_sigma`).

`estimate_gyro_bias: false` volta ao comportamento anterior (exceto a correção da covariância
zero, abaixo, que vale sempre).

**Covariância da IMU:** o driver do ZED-F9R publica `angular_velocity_covariance` com zeros.
Zero é tratado como desconhecido (usa `imu_yaw_rate_sigma`); antes, o filtro tomava o
giroscópio como perfeito e a incerteza do heading só diminuía.

Validação com o rosbag de 12/10/2026 (reprodução offline, mesmo bag nas duas versões):

| | Sem bias | Com bias |
|---|---|---|
| Deriva do heading parado | −0,219 °/s | +0,002 a +0,004 °/s (entre reproduções) |
| Parado, só integrando o giroscópio | −0,220 °/s | −0,020 °/s |
| Bias estimado (real medido parado ≈ −0,0035 rad/s) | — | −0,00325 → −0,00353 rad/s, estável andando |
| Erro de heading × rumo do GPS, andando (p5…p95) | −8,3° … +8,7° | −9,2° … +6,7° |

O erro de heading andando não muda: ele vem de outras fontes (modelo de esterçamento, rumo do
GPS), não do bias.

## Compilação

Depende do pacote de mensagens `vida_interfaces` (não incluído aqui — precisa estar no mesmo
workspace, ex. como submodule).

```bash
cd ~/ros2_ws   # ou o workspace onde o vida_interfaces estiver
source /opt/ros/humble/setup.bash
colcon build --packages-select vida_interfaces robot_state_ekf --symlink-install
source install/setup.bash
```

## Executando

```bash
ros2 launch robot_state_ekf robot_state_ekf.launch.py
```

Para trocar um parâmetro pontual sem editar o YAML:
```bash
ros2 launch robot_state_ekf robot_state_ekf.launch.py \
  params_file:=/caminho/outro_params.yaml
# ou, rodando o executável direto:
ros2 run robot_state_ekf robot_state_ekf_node_exec --ros-args \
  --params-file ~/ros2_ws/src/robot_state_ekf/config/robot_state_ekf_params.yaml \
  -p velocity_source:=rear
```

## Visualizando os tópicos

```bash
ros2 topic list                       # confirma que os topicos de entrada/saida existem
ros2 topic hz /robot_state/odom       # deve ficar em publish_rate_hz (10 Hz por padrao)
ros2 topic echo /robot_state/odom
ros2 topic echo /robot_state/steering_angle
```

Nada é publicado antes do EKF inicializar (origem GNSS fixada + heading estimado) — acompanhe o
log do nó (`RCLCPP_INFO`/`RCLCPP_WARN`) para ver esse processo:

```bash
ros2 topic hz /vida/sensors/gps/fix   # confirma que o GPS esta chegando, se a inicializacao travar
```

### Visualizando apenas a pose

```bash
ros2 topic echo /robot_state/odom --field pose.pose
```

Para só a posição (x, y) ou só a orientação, sem instalar nada além do `ros2topic`:
```bash
ros2 topic echo /robot_state/odom --field pose.pose.position
ros2 topic echo /robot_state/odom --field pose.pose.orientation
```

## Visualizando no RViz

```bash
rviz2
```

1. **Fixed Frame**: `odom`.
2. Add → By display type → **Odometry**, tópico `/robot_state/odom` (mostra a trajetória com
   setas de orientação).
3. Add → By display type → **TF**, para ver a transform `odom -> base_link` sendo publicada
   (exige `publish_tf: true`, que é o default).

Se o robô não tiver um modelo/URDF carregado, a TF sozinha já é suficiente para acompanhar a pose
estimada se movendo no plano.
