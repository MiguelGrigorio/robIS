# robis_odrive

Interface ROS 2 para os controladores ODrive do robô robIS e simulador de
locomoção diferencial.

## Dependências

- ROS 2 Jazzy;
- `rclpy`, `geometry_msgs`, `nav_msgs`, `tf2_ros`, `std_msgs` e `std_srvs`;
- pacote Python `odrive` para operação física;
- pacote Python `transforms3d`.

## Compilação

```bash
cd ~/robis_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robis_odrive
source install/setup.bash
```

## Execução

Robô real:

```bash
ros2 launch robis_odrive odrive_launch.py
```

Simulador isolado, sem hardware:

```bash
ros2 launch robis_odrive odrive_launch.py \
  simulation_mode:=true \
  cmd_vel_topic:=/cmd_vel \
  odom_topic:=/odom \
  publish_odom_tf:=false
```

Não execute o simulador ODrive junto com a odometria do Gazebo; deve existir
somente um publicador de `/odom` e de `odom → base_footprint` por ambiente.

## Interfaces

Assina:

- `/cmd_vel` (`geometry_msgs/msg/Twist`), configurável por `cmd_vel_topic`.

Publica:

- `/odom` (`nav_msgs/msg/Odometry`), configurável por `odom_topic`;
- `odom → base_footprint`, quando `publish_odom_tf` está habilitado;
- diagnósticos dos motores em tópicos `odrive/...`.

Serviço:

- `reset_odometry` (`std_srvs/srv/Trigger`).

## Parâmetros principais

- `simulation_mode`;
- `wheel_track`;
- `tyre_circumference`;
- `cmd_vel_topic`;
- `odom_topic`;
- `odom_frame`;
- `base_frame`;
- `publish_odom`;
- `publish_odom_tf`;
- `odom_calc_hz`.

## Troubleshooting

- Sem movimento: confirme a conexão USB com o ODrive e se `/cmd_vel` está recebendo mensagens.
- TF duplicado: desligue `publish_odom_tf` quando outro componente fornecer `odom → base_footprint`.
- Odometria invertida ou com escala incorreta: revise `wheel_track`, `tyre_circumference` e a calibração dos encoders.
- Erro de importação Python: instale as dependências do pacote no mesmo ambiente usado pelo workspace.
