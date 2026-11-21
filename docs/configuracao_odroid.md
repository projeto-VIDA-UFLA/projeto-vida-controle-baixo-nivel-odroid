# Configuração do Odroid: como refazer do zero

Tudo o que está configurado no Linux do Odroid do VIDA e não vem dos repositórios. Serve para
reconfigurar o sistema num cartão novo, caso o backup se perca.

Levantado em 01/11/2026, por SSH, a partir do sistema em uso e do histórico de comandos, só com leitura.
**O passo a passo ainda não foi testado num cartão novo.**

> Senhas, credenciais NTRIP e chaves ficam de fora de propósito. Elas estão listadas em
> [O que só existe no cartão](#o-que-só-existe-no-cartão).

Para operar o carro no dia a dia, veja a página do projeto: [projeto-VIDA-UFLA](https://github.com/projeto-VIDA-UFLA).

## Resumo do sistema

| Item | Valor |
|---|---|
| Placa | Hardkernel ODROID-N2 (arm64, 6 núcleos, 4 GB) |
| Sistema | Ubuntu 22.04.5 LTS, imagem mínima da Hardkernel, atualizada desde 2019 |
| Kernel | 4.9.337-38 (`linux-odroid-n2`) |
| Cartão | 59,5 GB: `mmcblk1p1` 128 MB vfat em `/media/boot`; `mmcblk1p2` ext4 em `/` (15 GB usados) |
| Swap | Nenhuma |
| Usuário | `odroid`, nos grupos `dialout`, `sudo` e `docker` |
| Hostname / fuso | `odroid` / UTC |
| Docker | 29.6.1, Compose 5.2.0 |
| PlatformIO | 6.1.19, em `~/.platformio`, com Python 3.10.12 |
| Tailscale | 1.102.2 |

ROS 2 não fica instalado no sistema: ele roda dentro dos containers.

## Passo a passo

### 1. Gravar a imagem
Grave a imagem Ubuntu mínima da Hardkernel para o ODROID-N2 e atualize o sistema. A partir de 27/06/2026,
o histórico mostra `apt upgrade` e depois as instalações abaixo.

Confira se o usuário `odroid` existe, porque os caminhos deste guia usam `/home/odroid`.

### 2. Instalar os pacotes
```bash
sudo apt update && sudo apt upgrade
sudo apt install -y git openssh-client openssh-server build-essential dkms bc cmake \
  python3 python3-venv python3-pip curl gnupg ca-certificates network-manager
```
O `cmake`, o `git` e o `python3-venv` são usados na compilação da libmicroros.

Pacotes extras da imagem que o projeto não usa diretamente (`mali-fbdev`, `aml-libs`, `cifs-utils`,
`libsdl2-2.0-0`) não entram na lista.

### 3. Instalar o Docker
```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=arm64 signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu jammy stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
```

### 4. Dar as permissões ao usuário
```bash
sudo usermod -aG dialout,docker odroid
```
Saia e entre de novo no SSH para valer.

### 5. Criar as regras udev
Elas criam os nomes fixos que o `docker-compose.yml` e o `platformio.ini` usam.

`/etc/udev/rules.d/99-vida-esp32.rules` (ESP32, conversor CP210x):
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="vida_esp32", MODE="0666", GROUP="dialout"
```
`/etc/udev/rules.d/99-vida-f9r.rules` (GPS ZED-F9R):
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="vida_f9r", GROUP="dialout", MODE="0660"
```
`/etc/udev/rules.d/99-vida-ublox.rules` (u-blox `01a8`; o `docker-compose.yml` atual não usa `/dev/vida_ublox`):
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="vida_ublox", GROUP="dialout", MODE="0660"
```
Depois:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
**Trocou o ESP32 ou o conversor USB?** A regra depende do serial `0001`. Para ver os valores do aparelho novo:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head -30
```

### 6. Configurar a rede
O Odroid sai para a internet pelo cabo, passando pelo notebook. O NetworkManager cuida da rede, e não há
arquivos em `/etc/netplan`.

```bash
sudo nmcli con add type ethernet con-name vida-ponto-a-ponto ifname eth0 autoconnect yes \
  ipv4.method manual ipv4.addresses 192.168.10.1/24 ipv4.gateway 192.168.10.2 \
  ipv4.dns "8.8.8.8 1.1.1.1" ipv4.route-metric 100 ipv4.may-fail no \
  connection.autoconnect-priority 100 ipv6.method disabled
sudo nmcli con up vida-ponto-a-ponto
```
Isso gera o arquivo `/etc/NetworkManager/system-connections/vida-ponto-a-ponto.nmconnection`, que hoje está assim:
```ini
[connection]
id=vida-ponto-a-ponto
uuid=2d2386ce-3f0a-3d10-a610-9450b39d2cff
type=ethernet
autoconnect-priority=100
interface-name=eth0

[ethernet]

[ipv4]
address1=192.168.10.1/24,192.168.10.2
dns=8.8.8.8;1.1.1.1;
may-fail=false
method=manual
route-metric=100

[ipv6]
addr-gen-mode=eui64
method=disabled

[proxy]
```
Para criar o arquivo à mão, use dono root e permissão `600`, e rode `sudo nmcli connection load <arquivo>`.
O `uuid` pode ser outro.

| Conexão | Interface | Endereço | Observação |
|---|---|---|---|
| `vida-ponto-a-ponto` | `eth0` | 192.168.10.1/24, gateway 192.168.10.2 | Cabo até o notebook |
| `LMT Local` | `wlan0` | DHCP | Wi-Fi com WPA-PSK, conexão automática. Precisa do adaptador USB |
| `tailscale0`, `docker0` | | | Criadas pelo Tailscale e pelo Docker |

**O notebook tem que usar `192.168.10.2`**: é o gateway e a saída para a internet configurados
acima, de onde vêm as correções RTK. Qualquer outro IP no cabo deixa a Odroid sem internet.

**Outros notebooks** não precisam da configuração manual abaixo: o
`scripts/rede/preparar_computador.sh` do alto-nível instala o NAT e o comando `vida-rede`, e o
perfil `192.168.10.2` só existe enquanto o botão **Conectar à Odroid** do painel (ou
`vida-rede conectar`) está ativo. Roteiro na página do projeto: [projeto-VIDA-UFLA](https://github.com/projeto-VIDA-UFLA).

**No notebook principal** (`enp4s0` no cabo, `wlp0s20f3` no Wi-Fi), a conexão cabeada fica com IP manual
`192.168.10.2/24`, e o NAT vem do script `/etc/NetworkManager/dispatcher.d/90-nat-odroid`
(dono root, executável):
```bash
#!/bin/bash
IFACE="$1"; ACTION="$2"
LAN=enp4s0
WAN=wlp0s20f3
REDE=192.168.10.0/24

[ "$IFACE" = "$LAN" ] || exit 0

regra() {  # $1 = A (adicionar) ou D (remover); o resto = regra
  op=$1; shift
  if [ "$op" = A ]; then
    iptables -C "$@" 2>/dev/null || iptables -I "$@"
  else
    iptables -D "$@" 2>/dev/null
  fi
}

case "$ACTION" in
  up)   OP=A; sysctl -w net.ipv4.ip_forward=1 ;;
  down) OP=D ;;
  *)    exit 0 ;;
esac

regra $OP POSTROUTING -t nat -s $REDE -o $WAN -j MASQUERADE
regra $OP FORWARD -i $LAN -o $WAN -s $REDE -j ACCEPT
regra $OP FORWARD -i $WAN -o $LAN -d $REDE -m state --state RELATED,ESTABLISHED -j ACCEPT
```
Sem esse script, o Odroid fica sem internet e sem correções RTK. Ele só aceita o Wi-Fi como saída.

#### Prioridade automática: cabo primeiro, Wi-Fi como reserva
Não há script fazendo a troca. Ela vem da **métrica das rotas**: com duas rotas padrão, o Linux usa a de
menor métrica.

| Conexão | `ipv4.route-metric` | Métrica efetiva | Papel |
|---|---|---|---|
| `vida-ponto-a-ponto` | `100` (definida à mão) | 100 | Preferida quando o cabo está conectado |
| `LMT Local` | `-1` (padrão) | 600 (padrão do NetworkManager para Wi-Fi) | Assume quando a rota do cabo sai |

As duas conexões têm `autoconnect` ligado, então podem ficar ativas ao mesmo tempo. Ao desconectar o cabo,
a rota de `eth0` sai e o tráfego vai para o Wi-Fi. Ao conectar de novo, a rota volta e o cabo retoma a
preferência. O `autoconnect-priority=100` só define qual perfil o NetworkManager escolhe primeiro.

**Limitação:** a troca depende do link do cabo, não de haver internet. Com o cabo conectado e o notebook sem
internet, o tráfego continua pelo cabo e não vai para o Wi-Fi. Não há verificação de conectividade
configurada no NetworkManager.

Para conferir com os dois conectados, `ip route` deve mostrar `metric 100` em `eth0` e `metric 600` em `wlan0`.
Esse teste não foi feito no levantamento, porque o adaptador Wi-Fi não estava conectado.

### 7. Instalar o driver do Wi-Fi (opcional)
O adaptador USB usa o driver `rtl8188fu/1.0`, instalado por DKMS. **A origem do código não aparece no
histórico.** É preciso achar o repositório do driver e confirmar se ele compila neste kernel. O carro
funciona sem isso pelo cabo.

O `cp210x` (ESP32) e o `cdc_acm` (GPS) já vêm no kernel.

### 8. Instalar o Tailscale (opcional)
Use o repositório oficial, `https://pkgs.tailscale.com/stable/ubuntu jammy main`, e depois:
```bash
sudo apt install -y tailscale tailscale-archive-keyring
sudo tailscale up
sudo tailscale set --accept-dns=false
```
O `--accept-dns=false` é necessário para o DNS continuar sendo o da conexão cabeada.

### 9. Configurar o Git e o acesso ao GitHub (opcional)
Os repositórios são públicos, então a chave SSH só é necessária para enviar alterações.
```bash
git config --global user.name "Nucleo-LMT"
git config --global user.email "<e-mail da conta do núcleo>"
ssh-keygen -t ed25519 -C "<e-mail da conta do núcleo>"
```
Cadastre a chave pública na conta `Nucleo-LMT`, ou restaure a chave do backup. Crie `~/.ssh/config`
para o GitHub usar a porta 443:
```
Host github.com
    Hostname ssh.github.com
    Port 443
    User git
    IdentityFile ~/.ssh/id_ed25519
```

### 10. Clonar os repositórios
```bash
cd ~
git clone https://github.com/projeto-VIDA-UFLA/projeto-vida-firmware-esp32.git
git clone https://github.com/projeto-VIDA-UFLA/projeto-vida-controle-baixo-nivel-odroid.git
```
O EKF (`robot_state_ekf`) já vem dentro do repositório da Odroid.

### 11. Criar o `.env`
Em `~/projeto-vida-controle-baixo-nivel-odroid/.env` (fora do Git, nunca commitar):
```
VIDA_NTRIP_USERNAME=<usuário NTRIP>
VIDA_NTRIP_PASSWORD=<senha NTRIP>
```

### 12. Instalar o PlatformIO
```bash
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
rm get-platformio.py
```

### 13. Acrescentar ao `~/.bashrc`
O projeto depende desta função: é ela que para o agente micro-ROS antes do upload e o reinicia depois.
```bash
export PATH="$PATH:$HOME/.platformio/penv/bin"

pio() {
    local is_run=false
    local is_upload=false
    local previous_arg=""
    local container_was_running=false
    local CONTAINER="micro_ros_agent"

    for arg in "$@"; do
        if [[ "$arg" == "run" ]]; then
            is_run=true
        fi

        if [[ "$previous_arg" == "--target" && "$arg" == "upload" ]]; then
            is_upload=true
        fi

        if [[ "$previous_arg" == "-t" && "$arg" == "upload" ]]; then
            is_upload=true
        fi

        if [[ "$arg" == "--target=upload" ]]; then
            is_upload=true
        fi

        previous_arg="$arg"
    done

    if [[ "$is_run" == true && "$is_upload" == true ]]; then
        if docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER"; then
            container_was_running=true

            echo "Parando o micro-ROS Agent antes do upload..."
            docker stop "$CONTAINER" || {
                echo "Erro ao parar o container $CONTAINER."
                return 1
            }
        else
            echo "O container $CONTAINER já está parado."
        fi
    fi

    command pio "$@"
    local pio_result=$?

    if [[ "$container_was_running" == true ]]; then
        echo "Reiniciando o micro-ROS Agent..."
        docker start "$CONTAINER" || {
            echo "Aviso: não foi possível reiniciar o container."
        }
    fi

    return "$pio_result"
}
```
O `~/.bashrc` atual também tem uma função `ros2()` que chama `docker exec -it ros_tools ...`. **Ela não
funciona hoje**, porque o container `ros_tools` não existe mais, então não é preciso copiá-la. Para usar
ROS 2 no Odroid:
```bash
docker exec -it ros2_ws bash -lc 'source /vida_ws/install/setup.bash; ros2 topic list'
```

### 14. Compilar e subir
```bash
cd ~/projeto-vida-controle-baixo-nivel-odroid
docker compose build

cd ~/projeto-vida-firmware-esp32
pio run -e vida_firmware          # 1ª vez: compila a libmicroros, demora
pio run -e vida_firmware -t upload

cd ~/projeto-vida-controle-baixo-nivel-odroid
docker compose up -d
```
Os containers usam `restart: always` e o serviço `docker` fica habilitado, então sobem sozinhos no boot
e voltam em caso de falha.

| Imagem | Origem |
|---|---|
| `vida-ros2-ws:humble` | `ros_ws/Dockerfile` (base `ros:humble`) |
| `vida-micro-ros-agent:humble` | `micro_ros_agent/Dockerfile` (base `microros/micro-ros-agent:humble`) |

### 15. Conferir
```bash
ls -l /dev/vida_esp32 /dev/vida_f9r           # com os aparelhos conectados
ip route                                      # default via 192.168.10.2
ping -c 3 8.8.8.8                             # internet pelo notebook
docker ps                                     # micro_ros_agent e ros2_ws rodando
docker compose logs --tail 20 micro_ros_agent # entidades do ESP32 criadas
dkms status                                   # só se usar o Wi-Fi
```

## O que só existe no cartão
Guarde estes itens junto com o backup, em local seguro e **fora dos repositórios**:

| Item | Onde | Sem ele |
|---|---|---|
| Credenciais NTRIP | `~/projeto-vida-controle-baixo-nivel-odroid/.env` | Sem correções RTK |
| Chave SSH do GitHub | `~/.ssh/id_ed25519` | Gerar outra e cadastrar na conta |
| Senha do Wi-Fi `LMT Local` | `/etc/NetworkManager/system-connections/` | Pedir a senha de novo |
| Login do Tailscale | `/var/lib/tailscale` | Autorizar o aparelho de novo |
| Senha do usuário `odroid` | Sistema | Definir outra |

## Serviços
Habilitados, e que importam para o projeto: `docker`, `containerd`, `ssh`, `NetworkManager`,
`systemd-resolved`, `systemd-timesyncd` e `tailscaled`.

Da imagem da Hardkernel: `odroid-tweaks` (afinidade de interrupções USB e Ethernet), a regra
`10-odroid.rules` e `cpufrequtils`.

Não há tarefas no `crontab` do usuário nem serviços próprios do projeto no systemd. Quem sobe o sistema
é o Docker.

## Pontos para revisar
Nada disso foi alterado. São observações do levantamento:

- **`unattended-upgrades` ligado:** o sistema se atualiza sozinho, e uma atualização pode mudar algo perto
  de um teste. O kernel 4.9 fica em uso, mas o DKMS mantém compilações para kernels 5.15 instalados.
- **Sobras:** imagens `vida-ros-ws` e `vida-ros-tools`, container `sweet_mccarthy`, regra `vida_ublox`,
  função `ros2()` e o arquivo vazio `.vida-ponto-a-ponto.nmconnection.swp`.
- **Comentário da chave SSH:** é o texto de exemplo `seu-email@gmail.com`. Não afeta o funcionamento.

## Outros itens conferidos

- **`iptables`:** só regras criadas pelo Docker e pelo Tailscale. O Odroid não tem regras próprias do projeto.
- **`sudoers`:** o padrão. Em `/etc/sudoers.d` só há o `README`.
- **Root:** sem `crontab`.
- **NetworkManager:** sem scripts próprios em `dispatcher.d`. O `plugins=ifupdown,keyfile` e os demais
  arquivos de `conf.d` são os da instalação. O Wi-Fi fica com `wifi.powersave = 3` (economia de energia ligada).
- **Boot:** `/media/boot/boot.ini` e `config.ini` têm o formato da Hardkernel. O `boot.ini` traz
  `net.ifnames=0`, que mantém o nome `eth0` usado na rede, e o UUID da partição `/`, próprio deste cartão.
  Não comparei os arquivos com os originais da imagem, então vale salvar uma cópia dos dois com o backup.
