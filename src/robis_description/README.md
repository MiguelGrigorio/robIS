# robis_description

Descrição do robô robIS para ROS 2 Jazzy.

## O que o pacote fornece

- Modelo `robis.urdf.xacro`.
- Parâmetros centralizados em `config/robot_parameters.yaml`.
- Malhas do chassi, rodas, borracha e marcador ArUco.
- `display.launch.py` para visualização no RViz2.
- Configuração RViz em `rviz/robis.rviz`.

## Estrutura

```text
robis_description/
├── CMakeLists.txt
├── package.xml
├── config/
├── launch/
├── meshes/
├── rviz/
└── urdf/
```

## Dependências e compilação

```bash
cd ~/robis_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robis_description
source install/setup.bash
```

## Execução

```bash
ros2 launch robis_description display.launch.py
```

Para usar os controles das juntas:

```bash
ros2 launch robis_description display.launch.py use_joint_state_publisher_gui:=true
```

## Frames e troubleshooting

- O frame principal da navegação é `base_footprint`; o chassi físico é `base_link`.
- Os sensores usam `lidar_link` e `camera_link`.
- Se o modelo não aparecer, confirme a instalação do pacote e execute `xacro` diretamente para localizar erros no arquivo.
