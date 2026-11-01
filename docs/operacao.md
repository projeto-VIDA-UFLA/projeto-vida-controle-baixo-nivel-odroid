# Operação do VIDA: do zero até o carro andando

Passo a passo completo, nas duas máquinas: atualizar os três repositórios, gravar o
firmware, ligar, rodar o carro no manual ou no automático (Stanley) e gravar uma rota nova.

> O mesmo arquivo está em `projeto-vida-controle-alto-nivel/docs/operacao.md` e em
> `projeto-vida-controle-baixo-nivel-odroid/docs/operacao.md`. Ao alterar, copiar para o outro.

| Máquina | Repositório | Branch de teste |
|---|---|---|
| Odroid (`ssh odroid@192.168.10.1`) | `~/projeto-vida-firmware-esp32` | `pendente-teste/ajustes-pre-merge` |
| Odroid | `~/projeto-vida-controle-baixo-nivel-odroid` | `pendente-teste/navegacao` |
| Notebook | `~/projeto-vida/projeto-vida-controle-alto-nivel` | `pendente-teste/navegacao` |

> **Faça as etapas 1 a 3 no laboratório, com internet.** O `clean_microros`
> baixa e recompila a libmicroros (alguns minutos), e os `docker compose build`
> também baixam pacotes. Na pista, só as etapas 4 em diante.

## 1. Odroid: atualizar e compilar

```bash
ssh odroid@192.168.10.1

# Baixo nível (agente micro-ROS, GPS, EKF, segurança)
cd ~/projeto-vida-controle-baixo-nivel-odroid
git branch --show-current                   # pendente-teste/navegacao
git pull
git submodule update --init --recursive     # robot_state_ekf
docker compose build
```

## 2. Odroid: gravar o firmware do ESP32

O agente micro-ROS ocupa a serial do ESP32. No Odroid, o `pio run -t upload` já
para o agente se ele estiver rodando e o reinicia depois do upload (configuração
local do Odroid), então não é preciso pará-lo à mão.

```bash
cd ~/projeto-vida-firmware-esp32
git branch --show-current                   # pendente-teste/ajustes-pre-merge
git pull

# Só quando alguma .msg ou o colcon.meta mudou (ex.: BatteryStatus, 31/10).
# Apaga e recompila a libmicroros: demora e precisa de internet.
pio run -e vida_firmware -t clean_microros
pio run -e vida_firmware

pio run -e vida_firmware -t upload
# Se o upload falhar ("Unable to verify flash chip connection"):
#   pio run -e vida_firmware -t upload --upload-option="--baud=115200"
```

## 3. Notebook: atualizar e compilar

```bash
cd ~/projeto-vida/projeto-vida-controle-alto-nivel
git branch --show-current                   # pendente-teste/navegacao
git pull
docker compose build

# Mensagens no ROS do próprio notebook (para ros2 bag/topic fora do container)
cd ros_ws && colcon build --packages-select vida_interfaces && cd ..
```

## 4. Ligar

**Odroid:**
```bash
cd ~/projeto-vida-controle-baixo-nivel-odroid
docker compose up -d                        # agente micro-ROS + ros2_ws (GPS, EKF, segurança)
docker compose logs -f micro_ros_agent      # Ctrl+C para sair; deve criar as entidades do ESP32
```

| Serviço (Odroid) | O que roda |
|---|---|
| `micro_ros_agent` | Agente micro-ROS na serial `/dev/vida_esp32` (921600). Espera 2 s antes de abrir a serial, para o ESP32 perceber a queda e reconectar |
| `ros2_ws` | `vida_odroid` (`vida.launch.py`): GPS/RTK (ZED-F9R em `/dev/vida_f9r`, correções NTRIP), `robot_state_ekf` e o nó `security` (enable lógico) |

O `.env` com o usuário e a senha do NTRIP fica só no Odroid, ao lado do
`docker-compose.yml` (está no `.gitignore`; nunca commitar).

**Notebook** (em todos os terminais com `ros2` no host):
```bash
source ~/projeto-vida/projeto-vida-controle-alto-nivel/ros_ws/install/setup.bash
```

Conferência rápida:
```bash
ros2 topic list | grep /vida                       # tópicos do ESP32, inclusive /vida/sensors/battery
ros2 topic echo /vida/safety/status --field inhibit_flags   # 0 = liberado
ros2 topic echo /vida/sensors/gps/status           # RTK FIXED antes de gravar rota ou entrar no automático
```

## 5a. Modo manual (gamepad)

```bash
cd ~/projeto-vida/projeto-vida-controle-alto-nivel
docker compose up -d                        # serviço ros_ws: só o gamepad
```

## 5b. Modo navegação (manual + Stanley)

O gamepad continua dirigindo no manual; o △ coloca o Stanley no comando.

```bash
cd ~/projeto-vida/projeto-vida-controle-alto-nivel
docker compose stop ros_ws
VIDA_PATH_FILE=/caminhos/2026-10-31/rota_4.csv docker compose --profile navegacao up -d navegacao
docker logs -f ros_ws_navegacao             # Ctrl+C para sair
```

`/caminhos` é a pasta `~/projeto-vida/analises` do notebook. De volta ao só manual:
```bash
docker compose stop navegacao && docker compose up -d ros_ws
```

## Comandos do gamepad

| Botão | Ação | Vibração |
|---|---|---|
| R2 / L2 | acelera (até 2,5 m/s) / freia | — |
| Analógico esquerdo | esterça (até 20°) | — |
| **R1 por 3 s** | modo manobra: freio solto para empurrar | 2 curtos: solto · 1 longo: freio de volta |
| **△ por 5 s** | entra no automático (só no modo navegação) | 1 pulso: aceito · 2 longos: Stanley no comando · 6 rápidos: recusado |
| □ por 5 s | sai do automático | 4 curtos |
| X / O por 5 s | entra / sai do modo bancada (veículo suspenso) | contínua / pulsada |

- **Manobra:** só entra com o carro parado. R2 ou L2 cancelam, assim como o carro passar de 2,5 m/s (freio total na hora; cuidado em descida) ou o enable físico ser desligado. O esterçamento funciona com o enable físico ligado.
- **Automático:** carro parado, RTK com correções e antena a até 3 m do início do caminho. Ele alinha as rodas, reinicia o EKF, anda reto até ter o heading e segue a volta. **L2 aborta a qualquer momento.**

## Gravar uma rota nova

Com o Odroid ligado (etapa 4), o notebook no modo manual (5a) e o carro **parado no
ponto de partida**, com RTK FIXED. Troque a data e o nome (`rota_5`).

**Terminal 1: reiniciar o EKF e gravar o bag** (reserva, caso o CSV tenha problema).
Fique parado uns 5 s depois do reset.
```bash
ros2 service call /robot_state_ekf_node/reset std_srvs/srv/Trigger
ros2 bag record -a -o ~/rosbag/rota_gravacao5
```

**Terminal 2: o gravador da rota**, logo em seguida. Dirija a volta e pare no ponto de partida; Ctrl+C salva.
```bash
mkdir -p ~/projeto-vida/analises/2026-11-01
docker exec -it ros_ws bash -c 'source /opt/ros/humble/setup.bash && source /vida_ws/install/setup.bash && ros2 run vida_navigation path_recorder --ros-args -p output_file:=/caminhos/2026-11-01/rota_5_bruta.csv -p closed:=true -p name:=rota_5'
```
- `closed:=false` para um trajeto de ida (começo e fim em lugares diferentes).
- Por padrão ele só aceita pontos com RTK FIXED. Na rota 4 (31/10, entre prédios) foi
  preciso `-p require_rtk_fixed:=false`; na pista aberta, deixe o padrão.

**Terminal 2: processar e validar.** Gera `rota_5.csv` e um `.png` ao lado, e mostra o raio mínimo:
```bash
docker exec ros_ws bash -c 'source /opt/ros/humble/setup.bash && source /vida_ws/install/setup.bash && ros2 run vida_navigation path_tools from-recording /caminhos/2026-11-01/rota_5_bruta.csv /caminhos/2026-11-01/rota_5.csv'
```

Depois, para rodar o Stanley nela: etapa 5b com `VIDA_PATH_FILE=/caminhos/2026-11-01/rota_5.csv`.

## Gravar um teste

```bash
ros2 bag record -a -o ~/rosbag/<teste>_<data>
```

O `-a` grava tudo, inclusive a bateria (`/vida/sensors/battery`). Sem o bag não dá para analisar o teste depois.

## Comandos úteis

```bash
ros2 service call /robot_state_ekf_node/reset std_srvs/srv/Trigger   # reinicia o EKF (carro parado)
ros2 topic echo /vida/sensors/battery                                 # bateria (valid: false sem o INA228)
docker compose restart micro_ros_agent                                # no Odroid: agente de novo (ESP32 reconecta em ~5 s)
docker compose logs -f ros2_ws                                        # no Odroid: GPS, EKF e segurança
```

## Mensagens (`vida_interfaces`)

O pacote existe nos três repositórios e precisa ter as mesmas definições:

| Repositório | Pasta |
|---|---|
| Firmware | `interfaces/vida_interfaces/` (só as mensagens que o ESP32 usa) |
| Odroid | `ros_ws/src/vida_interfaces/` |
| Notebook | `ros_ws/src/vida_interfaces/` |

Ao mudar uma `.msg`: mudar nos três e recompilar (`clean_microros` no firmware, etapa 2;
`docker compose build` no Odroid e no notebook; `colcon build` no notebook, etapa 3).

| Mensagem | Quem publica |
|---|---|
| `Encoders`, `Steering`, `Comands`, `SafetyStatus`, `BatteryStatus` | ESP32 |
| `SteeringReference`, `DriveReference` | notebook (gamepad / `mode_manager`) → ESP32 |
| `LogicalEnable` | Odroid (`security`) → ESP32 |
| `GnssStatus`, `EkfStatus` | Odroid |
| `StanleyStatus`, `NavigationMode` | notebook |

`BatteryStatus` (31/10): tensão, corrente e SoC do banco de tração, medidos pelo INA228
no ESP32, em `/vida/sensors/battery` a 10 Hz. Nenhum nó assina; serve para acompanhar e
gravar em bag. A corrente sai NaN até o shunt ser configurado no firmware.

## Problemas comuns

| Sintoma | O que fazer |
|---|---|
| Upload do ESP32 falha | Tentar com `--upload-option="--baud=115200"`; conferir com `docker ps` se o agente parou durante o upload |
| ESP32 não aparece depois de reiniciar o agente | Esperar ~5 s (o agente espera 2 s antes de abrir a serial); se não voltar, apertar o reset do ESP32 |
| Flag 4096, 512 ou 1024 | Falha retida: desligar e religar o enable físico para rearmar |
| Automático recusado (6 pulsos) | Ver o motivo em `docker logs ros_ws_navegacao` (distância do início, RTK, ESP32 bloqueado) |
| Bag sem `/vida/sensors/battery` | `colcon build --packages-select vida_interfaces` no notebook (etapa 3) e `source` de novo |
