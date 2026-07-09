# RobIS D2X ROS 2 Workspace

Workspace ROS 2 Jazzy do RobIS D2X. A fonte dos pacotes fica somente em
`src/`; a separacao entre robo real e simulacao acontece nos Dockerfiles.

## Estrutura

- `src/robis_bringup`: launchs de alto nivel para real, simulacao e demos.
- `src/robis_description`: URDF/Xacro, malhas e parametros fisicos do robo.
- `src/robis_gazebo`: mundo, bridge e launch do Gazebo.
- `src/robis_navigation`: SLAM, localizacao e navegacao.
- `src/robis_odrive`: driver ODrive e modo simulado do driver.
- `src/robis_ydlidar`: driver do YDLidar usado no robo real.
- `src/robis_vision`: camera USB e seguidor de bola por OpenCV.

## Imagens Docker

### Robo real

```bash
make build
```

`Dockerfile.robot` copia `src/`, mas mantem apenas pacotes necessarios no robo
real: bringup, description, navigation, ODrive, YDLidar e vision. Gazebo nao
entra nessa imagem.

### Simulacao

```bash
make build-sim
make run-sim
```

`Dockerfile.sim` usa a mesma pasta `src/`, mas remove `robis_ydlidar` antes do
build. O laser da simulacao vem do sensor do Gazebo publicado em
`/demo/scan`. O `robis_odrive` fica disponivel em `simulation_mode:=true` para
testes do node sem hardware.

## Topicos Na Simulacao

Na simulacao, os topicos do robo usam prefixo `/demo`:

- `/demo/cmd_vel`
- `/demo/odom`
- `/demo/scan`
- `/demo/camera/image_raw`
- `/demo/camera/camera_info`
- `/demo/joint_states`
- `/demo/tf`
- `/demo/tf_static`

O `/clock` continua global, como esperado pelo `use_sim_time`.

## Gazebo

```bash
ros2 launch robis_bringup simulation_full.launch.py stack:=slam
```

Para testar movimento direto:

```bash
ros2 topic pub /demo/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.2}}" -r 10
```

## ODrive E YDLidar Na Simulacao

O YDLidar real nao roda no Gazebo. Em simulacao, o equivalente funcional e o
sensor lidar do Gazebo, que publica `LaserScan` em `/demo/scan`.

O ODrive pode ser testado sem hardware:

```bash
ros2 launch robis_odrive odrive_launch.py \
  simulation_mode:=true \
  cmd_vel_topic:=/demo/cmd_vel \
  odom_topic:=/demo/odrive/odom \
  publish_odom_tf:=false
```

Para teste integrado de locomocao no Gazebo, normalmente use o DiffDrive do
Gazebo consumindo `/demo/cmd_vel`. Evite publicar duas odometrias principais ao
mesmo tempo; deixe `/demo/odom` vindo do Gazebo e use `/demo/odrive/odom` apenas
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
