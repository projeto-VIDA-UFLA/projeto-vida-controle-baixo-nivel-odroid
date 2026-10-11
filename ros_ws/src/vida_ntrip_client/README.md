# vida_ntrip_client

Cliente NTRIP v1 em C++17 para ROS 2 Humble. Ele recebe o stream RTCM da RBMC-IP por TCP e
publica os bytes, sem alteração, para o `vida_zed_f9r_driver`. O pacote é separado do driver GNSS:
uma falha de internet reinicia somente a conexão NTRIP e não encerra o F9R.

## Arquitetura

O nó `ntrip_client_node` tem uma thread ROS para publicações/diagnósticos e uma thread dedicada à
rede. A thread de rede usa `getaddrinfo()`, socket não bloqueante, `poll()` e um pipe de despertar
para interromper conexão, leitura ou backoff durante o shutdown. O cabeçalho NTRIP é analisado
incrementalmente; bytes RTCM recebidos no mesmo `recv()` do fim do cabeçalho são preservados.

O fluxo é publicado em `/vida/sensors/gps/rtcm` como `std_msgs/msg/UInt8MultiArray`, com QoS
confiável, `KeepLast(10)` e durabilidade volátil, compatível com o subscriber do F9R. Chunks TCP não
precisam coincidir com quadros RTCM. Em paralelo, um parser RTCM3 incremental calcula CRC-24Q e
contagens por tipo apenas para diagnóstico; ele nunca filtra ou reempacota os bytes publicados.

O launch `ntrip_client.launch.py` inicia somente o cliente. O launch
`zed_f9r_rtk.launch.py` inclui o launch original do `vida_zed_f9r_driver` e o cliente NTRIP; ele não
inicia o NEO-M8U.

## Parâmetros

| Parâmetro | Padrão | Descrição |
|---|---:|---|
| `caster_host` | `170.84.40.52` | host RBMC-IP |
| `caster_port` | `2101` | porta TCP |
| `mountpoint` | `MGLA0` | mountpoint sem necessidade de GGA |
| `rtcm_topic` | `/vida/sensors/gps/rtcm` | saída de bytes RTCM |
| `diagnostics_topic` | `/vida/sensors/gps/ntrip/diagnostics` | `DiagnosticArray` do cliente |
| `user_agent` | `NTRIP VIDA/1.0` | User-Agent NTRIP |
| `connect_timeout_ms` | `5000` | timeout de conexão |
| `header_timeout_ms` | `5000` | timeout do cabeçalho de resposta |
| `read_timeout_ms` | `15000` | máximo sem bytes do caster |
| `reconnect_initial_delay_ms` | `1000` | backoff inicial |
| `reconnect_max_delay_ms` | `30000` | teto do backoff exponencial |
| `max_response_header_bytes` | `8192` | limite defensivo do cabeçalho |
| `publish_chunk_max_bytes` | `4096` | tamanho máximo de cada mensagem ROS |
| `enable_rtcm_validation` | `true` | CRC/tipos apenas para diagnóstico |
| `send_gga` | `false` | envia GGA baseada no último `NavSatFix` |
| `fix_topic` | `/vida/sensors/gps/fix` | fonte opcional para GGA |
| `gga_interval_s` | `10.0` | intervalo de GGA após autenticação |

Todos os textos usados no cabeçalho rejeitam CR/LF, e portas, tempos, limites e intervalos são
validados antes de abrir a conexão. O modo GGA não inventa altitude: os campos sem medição ficam
vazios. Para `MGLA0`, mantenha `send_gga: false`.

## Credenciais seguras

Usuário e senha são lidos exclusivamente de `VIDA_NTRIP_USERNAME` e `VIDA_NTRIP_PASSWORD`. O nó
falha claramente se uma delas estiver vazia, mas não imprime valores nem o cabeçalho
`Authorization`. Não coloque a senha no YAML, launch, código ou Git. O `.env` local está ignorado;
`.env.example` contém somente os nomes vazios.

Exemplo sem gravar a senha no histórico e sem exigir o NEO-M8U:

```bash
export VIDA_NTRIP_USERNAME=whilker
read -rsp "Senha RBMC-IP: " VIDA_NTRIP_PASSWORD
echo
export VIDA_NTRIP_PASSWORD

docker compose run --rm \
  --name vida_f9r_rtk_test \
  -e VIDA_NTRIP_USERNAME \
  -e VIDA_NTRIP_PASSWORD \
  ros2_ws bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /vida_ws/install/setup.bash &&
    ros2 launch vida_ntrip_client zed_f9r_rtk.launch.py
  '

unset VIDA_NTRIP_PASSWORD
unset VIDA_NTRIP_USERNAME
```

O mapeamento `/dev/vida_f9r` já está no serviço principal `ros2_ws`; por isso não se passa
`--device` na linha de comando. Esse serviço inicia F9R + NTRIP pelo `vida.launch.py`; o NEO-M8U
antigo não faz mais parte do fluxo principal.

## Build e testes no container

Não execute `colcon` no host. A partir da raiz do repositório:

```bash
docker compose build ros2_ws

docker run --rm vida-ros2-ws:humble bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /vida_ws &&
  colcon build --packages-up-to vida_ntrip_client vida_zed_f9r_driver &&
  colcon test --packages-select vida_ntrip_client vida_zed_f9r_driver &&
  colcon test-result --verbose
'
```

Os testes usam um caster TCP em `127.0.0.1`; não acessam a RBMC-IP real. Eles cobrem Base64,
requisição e sanitização, respostas ICY/HTTP/erros, fragmentação do cabeçalho, limite, preservação
do corpo, GGA, CRC-24Q, ressincronização RTCM, tipos observados, reconexão e shutdown.

## Observação no host e diagnóstico

O Compose usa rede do host e Fast DDS UDP. No host, use uma instalação ROS 2 Humble e o mesmo
domínio do container (padrão `0`):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 topic hz /vida/sensors/gps/rtcm
ros2 topic echo /vida/sensors/gps/ntrip/diagnostics
ros2 topic echo /vida/sensors/gps/status
ros2 topic hz /vida/sensors/gps/fix
rqt_plot /vida/sensors/gps/fix/latitude /vida/sensors/gps/fix/longitude
```

O diagnóstico NTRIP informa `state` (`disconnected`, `connecting`, `authenticating`, `streaming`
ou `backoff`), conexão, idade do último byte, totais, conexões/reconexões, erro sanitizado, quadros
válidos/inválidos e contagens dos tipos 1006, 1008, 1013, 1019, 1020, 1033, 1074, 1084, 1094,
1124 e 1230.

Na mensagem real `vida_interfaces/msg/GnssStatus`, observe:

- `rtk_status=0` (`RTK_NONE`): sem solução RTK;
- `rtk_status=1` (`RTK_FLOAT`): solução diferencial com ambiguidades ainda não fixadas;
- `rtk_status=2` (`RTK_FIXED`): ambiguidades resolvidas;
- `corrections_received`: o receptor aceitou correções;
- `correction_age_s`: idade das correções conhecidas pelo driver/receptor.

Uma conexão NTRIP em `streaming` não garante automaticamente `RTK_FIXED`. O resultado depende de
céu aberto, antena GNSS dual-band adequada, geometria e visibilidade de satélites, disponibilidade
da estação base, distância da base e idade/continuidade das correções.

## Primeiro teste físico

1. Conecte o F9R em `/dev/vida_f9r`, posicione a antena dual-band com céu aberto e deixe o
   NEO-M8U desconectado para validar a independência.
2. Exporte as credenciais pelo procedimento acima e inicie `ros2_ws`.
3. Confirme `state=streaming` no diagnóstico e fluxo contínuo com `ros2 topic hz` no tópico RTCM.
4. Em `/vida/sensors/gps/status`, confirme `corrections_received: true` e
   `correction_age_s` baixo; os logs do driver também devem mostrar RXM-RTCM recebido/aceito.
5. Aguarde a transição `RTK_NONE` → `RTK_FLOAT` → `RTK_FIXED`, sem tratá-la como garantia.
6. Desconecte temporariamente a internet: o diagnóstico deve entrar em `backoff` e retornar a
   `streaming`, enquanto `/vida/sensors/gps/fix` continua sendo publicado pelo driver.
7. Procure nos logs somente por estados/erros; nenhuma credencial ou `Authorization` deve existir.

## Limitações

O caster configurado usa NTRIP v1 sobre TCP sem TLS, portanto a autenticação Basic não é cifrada na
rede. A resolução de nomes depende do resolvedor do sistema; o endereço RBMC-IP padrão é numérico.
O parser RTCM fornece observabilidade, não uma interpretação completa das mensagens. A aceitação
física e a obtenção de `RTK_FIXED` exigem hardware, credenciais e condições GNSS reais.
