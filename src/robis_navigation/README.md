# robis_navigation

Navegação, localização e SLAM para o robIS em ROS 2 Jazzy.

## O que este pacote entrega

- `slam.launch.py`: sobe o `slam_toolbox` para mapear o mundo a partir do `/scan` no real ou na simulação
- `localization.launch.py`: sobe localizacao 2D com `map_server` + `amcl`
- `navigation.launch.py`: sobe localizacao + stack completo do Nav2
- `config/nav2_params.yaml`: parametros base do Nav2 alinhados com `base_footprint`, `/odom` e `/scan`
- `config/nav2_params_demo.yaml`: variante de simulação com os mesmos tópicos canônicos do robô real
- `config/slam_toolbox.yaml`: parametros iniciais do SLAM 2D
- `config/ekf.yaml`: base inicial para futura fusao com `robot_localization`

## Ordem recomendada

1. Subir a simulacao:

```bash
ros2 launch robis_bringup simulation_full.launch.py stack:=slam
```

2. Mapear o ambiente:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
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

## Tópicos, frames e parâmetros

- O `blank_map.yaml` em `maps/` e apenas um placeholder para bootstrap.
- O `ekf.yaml` ainda nao entra no launch por padrao, porque a simulacao ja entrega `/odom` limpo e a fusao multi-sensorial vem depois.
- O pacote consome `/scan`, `/odom` e TF; publica os tópicos de mapa, custo e planejamento do Nav2.
- A cadeia mínima esperada é `map → odom → base_footprint → lidar_link`.
- `use_sim_time` deve ser `true` na simulação e `false` no robô real.

## Troubleshooting

- Sem mapa: confirme se o `slam_toolbox` recebe `/scan` e se o laser está conectado a `base_footprint`.
- Falha de localização: confirme o caminho do mapa e o frame `map`.
- Nav2 não inicia: verifique se todos os nós gerenciados aparecem como ativos e se apenas uma fonte publica `/odom`.
