# Workspace ROS 2 do robIS

Workspace ROS 2 Jazzy do robIS. A fonte dos pacotes fica somente em
`src/`; a separacao entre robo real e simulacao acontece nos Dockerfiles.

## Estrutura

- `src/robis_bringup`: launchs de alto nivel para real, simulacao e demos.
- `src/robis_description`: URDF/Xacro, malhas e parametros fisicos do robo.
- `src/robis_gazebo`: mundo, bridge e launch do Gazebo.
- `src/robis_navigation`: SLAM, localizacao e navegacao.
- `src/robis_odrive`: driver ODrive e modo simulado do driver.
- `src/robis_ydlidar`: driver do YDLidar, usado exclusivamente no robô real.
- `src/robis_vision`: camera USB e seguidor de bola por OpenCV.

## Imagens Docker

### Robo real

```bash
make build
```

`Dockerfile.robot` copia `src/`, mas mantem apenas pacotes necessarios no robo
real: bringup, description, navigation, ODrive, YDLidar e vision. Gazebo nao
entra nessa imagem.

### Simulacao com Docker Compose

```bash
make compose-build
make sim-up
```

O `docker-compose.yml` monta `src/` no container. Portanto, alteracoes nos
pacotes ficam disponiveis no proximo `sim-up` sem reconstruir a imagem; use
`make compose-build` quando mudar o `Dockerfile.sim` ou dependencias do
container.

Comandos uteis:

```bash
make sim-shell       # shell dentro do container
make sim-logs        # logs em tempo real
make sim-down        # encerra a simulacao
```

Antes da primeira execucao com Gazebo/RViz, habilite o acesso grafico:

```bash
xhost +local:docker
```

Se o host usar outro arquivo Xauthority, execute:

```bash
XAUTHORITY=/caminho/.Xauthority make sim-up
```

`Dockerfile.sim` usa a mesma pasta `src/` montada pelo Compose e o entrypoint
compila somente os pacotes necessários à simulação. O laser da simulação vem do sensor do Gazebo publicado em
`/scan`. O `robis_odrive` fica disponivel em `simulation_mode:=true` para
testes do node sem hardware quando for compilado explicitamente no workspace.

## Topicos Na Simulacao

Na simulação, os tópicos do robô usam os mesmos nomes canônicos do robô real:

- `/cmd_vel`
- `/odom`
- `/scan`
- `/camera/image_raw`
- `/camera/camera_info`
- `/joint_states`
- `/tf`
- `/tf_static`

O `/clock` continua global, como esperado pelo `use_sim_time`.

## Gazebo

```bash
ros2 launch robis_bringup simulation_full.launch.py stack:=slam
```

Para testar movimento direto:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.2}}" -r 10
```

## ODrive E YDLidar Na Simulacao

O YDLidar real nao roda no Gazebo. Em simulacao, o equivalente funcional e o
sensor lidar do Gazebo, que publica `LaserScan` em `/scan`.

O ODrive pode ser testado sem hardware:

```bash
ros2 launch robis_odrive odrive_launch.py \
  simulation_mode:=true \
  cmd_vel_topic:=/cmd_vel \
  odom_topic:=/odom \
  publish_odom_tf:=false
```

Para teste integrado de locomocao no Gazebo, normalmente use o DiffDrive do
Gazebo consumindo `/cmd_vel`. Evite publicar duas odometrias principais ao
mesmo tempo; deixe `/odom` vindo do Gazebo e use o ODrive apenas
para validar o node do ODrive.

## Seguir Bola

O mundo padrao do Gazebo tem uma bola laranja na frente do robo. Para testar a
camera simulada e o seguidor:

```bash
ros2 launch robis_bringup ball_follow_demo.launch.py
```

No robo real com Jetson Nano e webcam:

```bash
ros2 launch robis_bringup ball_follow_real.launch.py video_device:=/dev/video0
```

O node `ball_follower` detecta bola laranja por HSV e publica `Twist`. Ajustes
de cor, area minima e ganhos ficam em:

- `src/robis_vision/config/ball_follower.yaml`
- `src/robis_vision/config/ball_follower_demo.yaml`

## Deploy Na Jetson Nano

```bash
make setup
make deploy
```

Depois, na Jetson:

```bash
sudo docker run --runtime nvidia -it --rm \
  --network=host --privileged -v /dev:/dev \
  miguelgrigorio27/robis:jazzy \
  ros2 launch robis_bringup real_full.launch.py stack:=none
```
