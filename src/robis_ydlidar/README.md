# robis_ydlidar

Driver ROS 2 do lidar YDLidar usado exclusivamente no robô real robIS.

## Dependências

- ROS 2 Jazzy;
- SDK nativo YDLidar, localizado em `etc/YDLidar-SDK`;
- `rclcpp`, `sensor_msgs`, `std_srvs` e `tf2_ros`.

O SDK é compilado pelo `Dockerfile.robot` e instalado em `/opt/ydlidar-sdk`.
O pacote não deve ser compilado na imagem de simulação.

## Compilação no robô real

```bash
docker build -f Dockerfile.robot -t robis:robot .
```

Para um workspace já preparado manualmente, instale o SDK antes do build:

```bash
cmake -S etc/YDLidar-SDK -B /tmp/ydlidar-sdk-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/ydlidar-sdk
cmake --build /tmp/ydlidar-sdk-build -j"$(nproc)"
sudo cmake --install /tmp/ydlidar-sdk-build
colcon build --packages-select robis_ydlidar
```

## Execução

```bash
ros2 launch robis_ydlidar ydlidar_launch.py
```

O bringup completo do robô real inicia o driver por meio de
`robis_bringup/launch/real_full.launch.py` quando `enable_lidar:=true`.

## Interfaces

Publica:

- `/scan` (`sensor_msgs/msg/LaserScan`);
- `/point_cloud` (`sensor_msgs/msg/PointCloud`).

Serviços:

- `start_scan`;
- `stop_scan`.

TF publicado:

```text
base_footprint → lidar_link
```

## Parâmetros

Os parâmetros estão em `params/ydlidar.yaml`. Os principais são `port`,
`frame_id`, `baudrate`, `lidar_type`, `sample_rate`, `angle_min`, `angle_max`,
`range_min`, `range_max` e `frequency`.

## Troubleshooting

- Sem dados: confirme se o dispositivo existe (`ls -l /dev/ttyUSB*`) e se o
  container tem acesso privilegiado ao dispositivo.
- Falha de inicialização: confirme o modelo do lidar, `baudrate` e `lidar_type`.
- Erro de TF no RViz2: confirme que `frame_id` é `lidar_link` e que existe a
  transformação `base_footprint → lidar_link`.
