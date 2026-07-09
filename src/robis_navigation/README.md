# robis_navigation

Baseline de navegacao, localizacao e SLAM para o RobIS D2X em ROS 2 Jazzy.

## O que este pacote entrega

- `slam.launch.py`: sobe o `slam_toolbox` para mapear o mundo a partir do `/scan` no real ou `/demo/scan` com parametros demo
- `localization.launch.py`: sobe localizacao 2D com `map_server` + `amcl`
- `navigation.launch.py`: sobe localizacao + stack completo do Nav2
- `config/nav2_params.yaml`: parametros base do Nav2 alinhados com `base_footprint`, `/odom` e `/scan`
- `config/nav2_params_demo.yaml`: variante de simulacao usando `/demo/odom`, `/demo/scan` e `/demo/cmd_vel`
- `config/slam_toolbox.yaml`: parametros iniciais do SLAM 2D
- `config/ekf.yaml`: base inicial para futura fusao com `robot_localization`

## Ordem recomendada

1. Subir a simulacao:

```bash
ros2 launch robis_bringup simulation_full.launch.py stack:=slam
```

2. Mapear o ambiente:

```bash
ros2 topic pub /demo/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.2}}" -r 10
```

3. Mover o robô pela simulação e salvar o mapa:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/robis_ws/src/robis_navigation/maps/lab_map
```

4. Subir navegacao com o mapa salvo:

```bash
ros2 launch robis_bringup simulation_full.launch.py stack:=navigation map:=/home/ubuntu/robis_ws/src/robis_navigation/maps/lab_map.yaml
```

## Observacoes

- O `blank_map.yaml` em `maps/` e apenas um placeholder para bootstrap.
- O `ekf.yaml` ainda nao entra no launch por padrao, porque a simulacao ja entrega `/odom` limpo e a fusao multi-sensorial vem depois.
