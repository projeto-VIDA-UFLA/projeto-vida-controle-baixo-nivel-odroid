# vida_ublox_driver

Pacote ROS 2 Humble para o u-blox NEO-M8U-0-10 conectado por USB.

Publica:

- `/vida/sensors/gps/fix` (`sensor_msgs/msg/NavSatFix`) a partir de `UBX-NAV-PVT`.
- `/vida/sensors/imu/raw` (`sensor_msgs/msg/Imu`) a partir de `UBX-ESF-RAW`.

O nó habilita automaticamente as duas mensagens UBX na interface pela qual está
conectado. A orientação do `sensor_msgs/Imu` não é fornecida no tópico raw e sua
covariância é marcada com `-1`, conforme a convenção ROS.

## Estrutura no projeto

Copie a pasta `vida_ublox_driver` para:

```text
ros_ws/src/vida_ublox_driver
```

Substitua o Dockerfile de `ros_ws` pelo `Dockerfile.example` deste pacote, ou
adicione ao seu Dockerfile os pacotes `python3-serial` e
`ros-humble-sensor-msgs`.

## Dispositivo USB

Confira o receptor:

```bash
lsusb
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Para o USB nativo do NEO-M8U, o identificador esperado é `1546:01a8`. Há uma
regra de exemplo em `udev/99-vida-ublox.rules`, que cria o link estável
`/dev/vida_ublox`.

Instalação da regra no host:

```bash
sudo cp udev/99-vida-ublox.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/vida_ublox
```

Se o seu adaptador apresentar outro VID/PID, ajuste a regra conforme o resultado
de `udevadm info -a -n /dev/ttyACM0`.

## Docker Compose

No serviço `ros_ws`, acrescente:

```yaml
devices:
  - /dev/vida_ublox:/dev/vida_ublox
```

O arquivo `docker-compose.service-example.yml` mostra um serviço completo.

## Compilar e executar

```bash
docker compose build ros_ws
docker compose up -d ros_ws
docker compose exec ros_ws ros2 launch vida_ublox_driver ublox_m8u.launch.py
```

Verificação:

```bash
docker compose exec ros_ws ros2 topic hz /vida/sensors/gps/fix
docker compose exec ros_ws ros2 topic echo /vida/sensors/gps/fix --once
docker compose exec ros_ws ros2 topic hz /vida/sensors/imu/raw
docker compose exec ros_ws ros2 topic echo /vida/sensors/imu/raw --once
```

## Parâmetros

Edite `config/ublox_m8u.yaml` ou sobrescreva pela linha de comando:

```bash
ros2 run vida_ublox_driver ublox_m8u_node --ros-args \
  -p device:=/dev/ttyACM0 \
  -p baudrate:=115200
```

Na interface USB CDC, o baud rate não altera a velocidade física do USB, mas é
mantido como parâmetro para também permitir o uso por UART/USB-serial.

## Observações do M8U

- O `UBX-ESF-RAW` entrega força específica do acelerômetro e taxa angular bruta.
- A frequência do IMU é determinada pelo receptor. Habilitar a mensagem com
  taxa diferente de zero faz o M8U enviar todas as amostras disponíveis.
- O frame raw é o frame físico do IMU. Defina corretamente a transformação de
  `imu_link` para `base_link` de acordo com a montagem no veículo.
- Para dados inerciais compensados pelo filtro interno, pode-se acrescentar o
  parsing de `UBX-ESF-INS`; ele não substitui o tópico raw deste pacote.

