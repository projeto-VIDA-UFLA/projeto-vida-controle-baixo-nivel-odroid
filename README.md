# Projeto VIDA: nível embarcado (Odroid N2)

Software do computador embarcado **Odroid N2** do **VIDA (Veículo Inteligente de Desenvolvimento
Aplicado)**, protótipo de veículo autônomo do Laboratório de Mobilidade Terrestre (LMT) da UFLA.

A Odroid faz a ponte entre o ESP32 e o ROS 2, lê o receptor GNSS RTK, recebe as correções pela
internet, estima o estado do veículo (EKF) e gera o enable lógico enviado ao ESP32. Tudo roda
em ROS 2 Humble, em containers Docker que sobem sozinhos quando a Odroid liga.

- **Configurar a Odroid do zero:** [`docs/configuracao_odroid.md`](docs/configuracao_odroid.md)
- **Visão geral do projeto e como operar o veículo:** [projeto-VIDA-UFLA](https://github.com/projeto-VIDA-UFLA)

## Lugar na arquitetura

| Nível | Hardware | Repositório |
|---|---|---|
| 1 | ESP32 | [projeto-vida-firmware-esp32](https://github.com/projeto-VIDA-UFLA/projeto-vida-firmware-esp32): sensores, controle de baixo nível e segurança |
| 2 | **Odroid N2** | **Este repositório:** agente micro-ROS, GNSS RTK, NTRIP, EKF e enable lógico |
| 3 | Notebook | [projeto-vida-controle-alto-nivel](https://github.com/projeto-VIDA-UFLA/projeto-vida-controle-alto-nivel): Stanley, gerenciador de modos e gamepad; [projeto-vida-painel](https://github.com/projeto-VIDA-UFLA/projeto-vida-painel): painel de operação |

## Estrutura do repositório

```text
docker-compose.yml           Serviços micro_ros_agent e ros2_ws (reinício automático)
micro_ros_agent/             Imagem do agente micro-ROS
ros_ws/
  Dockerfile                 Imagem do ROS 2 Humble com os pacotes abaixo
  config/fastdds_udp.xml     Fast DDS só por UDP (comunicação com o notebook)
  src/
    vida_zed_f9r_driver/     Driver do receptor u-blox ZED-F9R (GNSS RTK e IMU)
    vida_ntrip_client/       Cliente NTRIP: correções RTK pela internet
    robot_state_ekf/         Filtro de Kalman estendido (posição, rumo e viés do giroscópio)
    vida_odroid/             Launch principal e nó security (enable lógico)
    vida_interfaces/         Mensagens ROS 2 do VIDA
docs/
  configuracao_odroid.md     Configuração do Linux da Odroid, do zero
```

Cada pacote tem a própria documentação:
[`vida_zed_f9r_driver`](ros_ws/src/vida_zed_f9r_driver/README.md),
[`vida_ntrip_client`](ros_ws/src/vida_ntrip_client/README.md) e
[`robot_state_ekf`](ros_ws/src/robot_state_ekf/README.md).

## Serviços

| Serviço | O que roda |
|---|---|
| `micro_ros_agent` | Agente micro-ROS na serial `/dev/vida_esp32` (921600 baud). Espera 2 s antes de abrir a serial, para o ESP32 perceber a queda e reconectar |
| `ros2_ws` | `vida.launch.py`: driver do ZED-F9R (`/dev/vida_f9r`), cliente NTRIP, `robot_state_ekf` e nó `security` |

Os dois serviços usam `restart: always`: sobem com a Odroid e voltam sozinhos em caso de falha.

## Tópicos principais

| Tópico | Mensagem | Origem |
|---|---|---|
| `/vida/sensors/gps/fix` | `sensor_msgs/NavSatFix` (2 Hz) | Driver ZED-F9R |
| `/vida/sensors/gps/status` | `GnssStatus` (estado RTK, satélites, idade das correções) | Driver ZED-F9R |
| `/vida/sensors/gps/nav_pvt` | `NavPvt` (solução completa do receptor) | Driver ZED-F9R |
| `/vida/sensors/imu/data_raw` | `sensor_msgs/Imu` (~100 Hz) | Driver ZED-F9R |
| `/vida/sensors/gps/rtcm` | Correções RTCM | Cliente NTRIP |
| `/vida/sensors/gps/ntrip/diagnostics` | Estado da conexão NTRIP | Cliente NTRIP |
| `/robot_state/odom` | `nav_msgs/Odometry` (50 Hz) | EKF |
| `/robot_state/status` | `EkfStatus` (fase, origem, estado RTK) | EKF |
| `/vida/safety/logical_enable` | `LogicalEnable` | `security`, a partir de `/gamepad/enabled` |

Serviço do EKF: `/robot_state_ekf_node/reset` (reinicia a origem e o viés, com o veículo parado).

## Compilar e subir

Na Odroid, na raiz do repositório:

```bash
docker compose build        # compila as imagens (precisa de internet)
docker compose up -d        # sobe os dois serviços
docker compose logs -f      # acompanha os logs (Ctrl+C para sair)
```

**Credenciais NTRIP:** o cliente lê `VIDA_NTRIP_USERNAME` e `VIDA_NTRIP_PASSWORD` de um arquivo
`.env` ao lado do `docker-compose.yml`. Esse arquivo existe só na Odroid e está no `.gitignore`:
nunca o envie ao repositório.

As mensagens de `vida_interfaces` precisam ter a mesma definição no firmware do ESP32 e no
notebook.

## Licença

MIT. Veja [`LICENSE`](LICENSE). O `robot_state_ekf` foi originalmente desenvolvido em um
repositório próprio e incorporado a este como pasta comum.
