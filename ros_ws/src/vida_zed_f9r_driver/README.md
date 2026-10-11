# vida_zed_f9r_driver

Driver ROS 2 Humble em C++17 para publicar as medições GNSS RTK e IMU bruta da placa
u-blox C102-F9R/ZED-F9R. Este pacote não implementa EKF, odometria, NTRIP nem usa as saídas
fusionadas `NAV-ATT`/`ESF-INS`. Por padrão, desativa em RAM tanto a fusão interna quanto o modo
de navegação prioritária.

## Hardware validado

- USB `1546:01a9`, USB CDC ACM
- C102-F9R com módulo `ZED-F9R`
- Firmware `HPS 1.30`, protocolo UBX `33.30`
- GPS, GLONASS, Galileo e BeiDou
- Regra udev esperada: `/dev/vida_f9r -> /dev/ttyACM0`

Os layouts, escalas e chaves vêm da
[Interface HPS 1.30 UBX-22010984](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPS-1.30_InterfaceDescription_UBX-22010984.pdf)
e do
[manual de integração ZED-F9R UBX-20039643](https://content.u-blox.com/sites/default/files/ZED-F9R_Integrationmanual_UBX-20039643.pdf).

## Build e execução

O projeto é compilado somente no container ROS 2 Humble:

```bash
docker compose build ros2_ws
docker compose run --rm ros2_ws bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /vida_ws &&
  colcon build --packages-up-to vida_zed_f9r_driver
'
```

Para iniciar somente este driver com a placa:

```bash
docker compose run --rm ros2_ws bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /vida_ws/install/setup.bash &&
    ros2 launch vida_zed_f9r_driver zed_f9r.launch.py
  '
```

`docker compose run` herda automaticamente os dispositivos declarados no serviço `ros2_ws`;
nessa versão do Compose ele não aceita a opção `--device`. Se apenas a F9R estiver conectada e o
Compose reclamar da ausência de `/dev/vida_ublox`, execute a mesma imagem sem herdar o
dispositivo antigo:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --device=/dev/vida_f9r:/dev/vida_f9r \
  --env FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/udp-only.xml \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  --env ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  --volume "$PWD/ros_ws/config/fastdds_udp.xml:/etc/fastdds/udp-only.xml:ro" \
  vida-ros2-ws:humble bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /vida_ws/install/setup.bash &&
    ros2 launch vida_zed_f9r_driver zed_f9r.launch.py
  '
```

Não execute simultaneamente `vida_ublox_driver`: ambos publicam um fix GNSS no tópico VIDA,
embora permaneçam pacotes separados e usem dispositivos diferentes.

## Visualização e plots no host

O serviço usa a rede do host e Fast DDS por UDP. No terminal do host que será usado para observar
ou plotar, selecione o mesmo domínio do container antes de iniciar o driver:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
```

Inicie o driver em outro terminal com as mesmas variáveis exportadas. O Compose repassa os
valores ao container. Depois, no host:

```bash
ros2 topic list
ros2 topic hz /vida/sensors/imu/data_raw
ros2 topic hz /vida/sensors/gps/fix
ros2 topic echo /vida/sensors/gps/fix --once
rqt_plot /vida/sensors/imu/data_raw/angular_velocity/x \
  /vida/sensors/imu/data_raw/angular_velocity/y \
  /vida/sensors/imu/data_raw/angular_velocity/z
```

Para aceleração, use no `rqt_plot` os campos `linear_acceleration/x`, `y` e `z`. Para GNSS,
podem ser plotados `/vida/sensors/gps/fix/latitude`, `longitude` e `altitude`. Esses tópicos usam
mensagens padrão de `sensor_msgs`, portanto não exigem instalar `vida_interfaces` no host. Para
decodificar `/vida/sensors/gps/status` diretamente no host, o ambiente ROS do host precisa ter a
nova versão de `vida_interfaces`; alternativamente, faça o `ros2 topic echo` dentro da imagem.

Se os tópicos não aparecerem, confirme que host e container usam o mesmo `ROS_DOMAIN_ID`, que
`ROS_LOCALHOST_ONLY` tem o mesmo valor nos dois ambientes e reinicie o daemon após alterar as
variáveis.

## Tópicos

| Direção | Tópico | Tipo | Conteúdo |
|---|---|---|---|
| saída | `/vida/sensors/gps/fix` | `sensor_msgs/msg/NavSatFix` | `NAV-HPPOSLLH`, estado de `NAV-PVT` e covariância `NAV-COV`/acurácias |
| saída | `/vida/sensors/gps/status` | `vida_interfaces/msg/GnssStatus` | fix, RTK, satélites, acurácias e correções |
| saída | `/vida/sensors/imu/data_raw` | `sensor_msgs/msg/Imu` | acelerômetro e giroscópio de `ESF-RAW` |
| entrada | `/vida/sensors/gps/rtcm` | `std_msgs/msg/UInt8MultiArray` | bytes RTCM sem alteração |

`rtk_status=1` significa RTK Float e `rtk_status=2`, RTK Fixed. Sem correções, é esperado
`rtk_status=0`; ainda é possível validar fix autônomo, satélites e IMU. `correction_age_s`
representa o limite inferior da faixa categórica informada por `NAV-PVT`; quando essa faixa não
está disponível e `RXM-RTCM` está habilitada, representa o tempo desde o último RTCM válido
observado. Sem informação, seu valor é `NaN`.

Um nó NTRIP separado deve publicar cada bloco recebido em `/vida/sensors/gps/rtcm`. O driver é
o único proprietário de `/dev/vida_f9r`, rejeita blocos vazios e serializa RTCM com as escritas UBX.

## Parâmetros

| Parâmetro | Padrão | Descrição |
|---|---:|---|
| `device` | `/dev/vida_f9r` | dispositivo serial/USB |
| `frame_id` | `f9r_link` | frame comum de GNSS e IMU |
| `baudrate` | `115200` | útil para UART; não define a taxa física do USB CDC |
| `reconnect_interval_ms` | `1000` | intervalo entre reconexões |
| `configuration_ack_timeout_ms` | `2000` | espera assíncrona por ACK/NAK |
| `disable_internal_sensor_fusion` | `true` | grava `CFG-SFCORE-USE_SF=false` somente em RAM |
| `priority_navigation_rate_hz` | `0` | `CFG-RATE-NAV_PRIO`; zero desativa o modo prioritário, faixa válida 0–30 Hz |
| `apply_receiver_configuration` | `true` | aplica `CFG-VALSET` no início/reconexão |
| `publish_raw_imu` | `true` | publica `ESF-RAW` |
| `publish_gnss` | `true` | publica fix e estado GNSS |
| `enable_rtcm_diagnostics` | `true` | habilita `UBX-RXM-RTCM` na saída USB somente em RAM |
| `correction_received_timeout_s` | `10.0` | validade local do último RTCM confirmado |
| `imu_axis_mapping` | `[1, 2, 3]` | permutação assinada dos eixos nativos para ROS |
| `gps_fix_topic` | `/vida/sensors/gps/fix` | tópico de fix |
| `gps_status_topic` | `/vida/sensors/gps/status` | tópico de estado |
| `imu_topic` | `/vida/sensors/imu/data_raw` | tópico da IMU bruta |
| `rtcm_topic` | `/vida/sensors/gps/rtcm` | entrada RTCM |

O mesmo `CFG-VALSET` contém `CFG-SFCORE-USE_SF` e `CFG-RATE-NAV_PRIO`, sempre com
`layers=0x01` (RAM): BBR e Flash nunca são selecionadas. A aplicação conjunta é validada por
`UBX-ACK-ACK`; `UBX-ACK-NAK` e timeout são registrados sem parar a recepção. Um poll
`UBX-MON-VER` registra software, hardware, `FWVER`, `PROTVER` e `MOD` a cada conexão.

## Frame da IMU e timestamp

`ESF-RAW` fornece força específica e velocidade angular no frame instrumental da IMU. O padrão
identidade não presume rotação: a placa deve estar rigidamente montada com o eixo x nativo para
a frente, y nativo para a esquerda e z nativo para cima, atendendo REP-103. Para outra montagem,
`imu_axis_mapping` define cada eixo ROS por índice nativo assinado; por exemplo, `[2, -1, 3]`
produz `ros_x=nativo_y`, `ros_y=-nativo_x`, `ros_z=nativo_z`. Isso é apenas permutação/inversão,
sem calibração ou atitude calculada. A orientação não é publicada e
`orientation_covariance[0] = -1`.

Nesta versão, `iTOW` e `sTtag` são preservados durante o pareamento, mas não são tratados como
Unix time. Fix e estado da mesma época `iTOW` recebem o mesmo horário ROS de recepção. Cada
amostra IMU completa recebe `node->now()` quando publicada. Uma conversão GNSS→ROS ficará
para uma etapa posterior.

## Testes

```bash
docker compose run --rm ros2_ws bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /vida_ws &&
  source install/setup.bash &&
  colcon test --packages-select vida_zed_f9r_driver &&
  colcon test-result --verbose
'
```

Para um futuro teste com NTRIP, inicie um cliente separado, publique RTCM no tópico de entrada e
observe `differential_solution`, `rtk_status`, `corrections_received` e as acurácias. Confirme
primeiro `RTK_FLOAT` e depois `RTK_FIXED`, sem alterar a configuração persistente do receptor.

## Verificação física do modo prioritário

Com `priority_navigation_rate_hz: 0`, o log de inicialização deve registrar um único
`UBX-ACK-ACK` para o `CFG-VALSET` conjunto e `navegacao prioritaria=0 Hz`. No host, meça:

```bash
ros2 topic hz /vida/sensors/gps/fix
```

O tópico deixa de usar a taxa prioritária configurada anteriormente em 10 Hz e passa a seguir a
taxa normal determinada por `CFG-RATE-MEAS` e `CFG-RATE-NAV`. Se essa taxa normal também for
10 Hz, a frequência observada continuará em 10 Hz e não distingue os modos; nesse caso, o ACK
confirma a aplicação do `CFG-VALSET`, mas uma confirmação independente exige consultar
`CFG-RATE-NAV_PRIO` com `UBX-CFG-VALGET`.
