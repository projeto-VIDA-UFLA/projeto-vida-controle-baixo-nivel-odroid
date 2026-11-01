# Operação do Odroid

Parte do Odroid no passo a passo do VIDA. O guia completo (notebook, gamepad,
Stanley e gravação de rota) fica no repositório do alto-nível:
`projeto-vida-controle-alto-nivel/docs/operacao.md`.

| Repositório no Odroid | Branch de teste |
|---|---|
| `~/projeto-vida-controle-baixo-nivel-odroid` (este) | `pendente-teste/navegacao` |
| `~/projeto-vida-firmware-esp32` | `pendente-teste/ajustes-pre-merge` |

> Atualize e compile **com internet** (no laboratório): o `docker compose build`
> e o `clean_microros` baixam pacotes.

## 1. Atualizar e compilar

```bash
ssh odroid@192.168.10.1

cd ~/projeto-vida-controle-baixo-nivel-odroid
git branch --show-current                   # pendente-teste/navegacao
git pull
git submodule update --init --recursive     # robot_state_ekf
docker compose build
```

## 2. Gravar o firmware do ESP32

```bash
cd ~/projeto-vida-firmware-esp32
git branch --show-current                   # pendente-teste/ajustes-pre-merge
git pull

# Só quando alguma .msg ou o colcon.meta mudou (ex.: BatteryStatus, 31/10).
# Apaga e recompila a libmicroros: demora e precisa de internet.
pio run -e vida_firmware -t clean_microros
pio run -e vida_firmware

pio run -e vida_firmware -t upload
# Se falhar ("Unable to verify flash chip connection"):
#   pio run -e vida_firmware -t upload --upload-option="--baud=115200"
```

O agente micro-ROS ocupa a serial do ESP32. Neste Odroid o `pio run -t upload`
já para o agente, se estiver rodando, e o reinicia depois do upload
(configuração local, fora dos repositórios).

## 3. Ligar

```bash
cd ~/projeto-vida-controle-baixo-nivel-odroid
docker compose up -d                        # micro_ros_agent + ros2_ws (GPS, EKF, segurança)
docker compose logs -f micro_ros_agent      # Ctrl+C para sair; deve criar as entidades do ESP32
```

| Serviço | O que roda |
|---|---|
| `micro_ros_agent` | Agente micro-ROS na serial `/dev/vida_esp32` (921600). Espera 2 s antes de abrir a serial, para o ESP32 perceber a queda e reconectar |
| `ros2_ws` | `vida_odroid` (`vida.launch.py`): GPS/RTK (ZED-F9R em `/dev/vida_f9r`, NTRIP pelo `.env`), `robot_state_ekf` e o nó `security` (enable lógico) |

O `.env` com o usuário e a senha do NTRIP fica só no Odroid (no `.gitignore`).

## Mensagens (`vida_interfaces`)

`ros_ws/src/vida_interfaces/` precisa ter as mesmas definições do notebook
(`projeto-vida-controle-alto-nivel/ros_ws/src/vida_interfaces/`) e, nas mensagens
que o ESP32 usa, do firmware (`projeto-vida-firmware-esp32/interfaces/vida_interfaces/`).
Ao mudar uma `.msg`, mudar nos três e recompilar: `docker compose build` aqui e no
notebook, `clean_microros` no firmware.

| Mensagem | Quem publica |
|---|---|
| `Encoders`, `Steering`, `Comands`, `SafetyStatus`, `BatteryStatus` | ESP32 |
| `SteeringReference`, `DriveReference`, `LogicalEnable` | notebook / Odroid → ESP32 |
| `GnssStatus`, `EkfStatus` | Odroid |
| `StanleyStatus`, `NavigationMode` | notebook |

`BatteryStatus` (31/10): tensão, corrente e SoC do banco de tração, medidos pelo
INA228 no ESP32, em `/vida/sensors/battery` a 10 Hz. Nenhum nó assina; serve para
acompanhar e gravar em bag.

## Comandos úteis

```bash
ros2 service call /robot_state_ekf_node/reset std_srvs/srv/Trigger   # reinicia o EKF (carro parado)
ros2 topic echo /vida/safety/status --field inhibit_flags             # 0 = liberado
ros2 topic echo /vida/sensors/gps/status                              # RTK FIXED?
docker compose restart micro_ros_agent                                # agente de novo (ESP32 reconecta em ~5 s)
```
