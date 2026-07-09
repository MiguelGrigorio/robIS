# maps

Mapas gerados pelo `slam_toolbox` e salvos com `nav2_map_server`.

## Arquivos iniciais

- `blank_map.yaml`: mapa placeholder para bootstrap do Nav2
- `blank_map.pgm`: imagem do mapa placeholder

## Fluxo recomendado

1. Suba a simulacao com `robis_gazebo`
2. Rode `ros2 launch robis_navigation slam.launch.py`
3. Mova o robô para cobrir o ambiente
4. Salve o mapa:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/robis_ws/src/robis_navigation/maps/lab_map
```

5. Rode:

```bash
ros2 launch robis_navigation navigation.launch.py map:=~/robis_ws/src/robis_navigation/maps/lab_map.yaml
```
