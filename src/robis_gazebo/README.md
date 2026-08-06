# robis_gazebo

Simulação Gazebo Sim do robô robIS para ROS 2 Jazzy.

## O que o pacote fornece

- Um mundo interno simples, inspirado nas demonstrações de navegação do TurtleBot3
- Launch files para abrir o Gazebo Sim e inserir o robô
- Uma configuração de bridge para os principais tópicos do projeto:
  - `/clock`
  - `/cmd_vel`
  - `/odom`
  - `/scan`
  - `/camera/image_raw`
  - `/camera/camera_info`
  - `/tf`

## Dependências, compilação e execução

```bash
cd ~/robis_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robis_description robis_gazebo
source install/setup.bash
ros2 launch robis_gazebo simulation.launch.py
```

## Verificações iniciais

Depois de iniciar, valide:

```bash
ros2 topic list | grep -E '/(odom|scan|camera|cmd_vel|tf)'
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /camera/camera_info --once
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.2}}' -r 10
```

## Frames e troubleshooting

- O pacote usa Gazebo Sim e `ros_gz_sim`.
- O modelo é gerado de `robis_description/urdf/robis.urdf.xacro` com `sim_flavor:=gz`.
- O mundo padrão é `turtlebot3_like.world`; `empty.world` pode ser usado para testes mínimos.
- Os tópicos simulados são os mesmos do robô real. Se o laser não aparecer, verifique `/scan` e a conexão TF até `lidar_link`.
