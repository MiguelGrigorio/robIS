# robis_bringup

Camada de integracao do projeto para manter o mesmo stack de localizacao e navegacao entre simulacao e robo real.

## Ideia central

O contrato do projeto fica assim:

- `robis_description`: descreve o robo e sua arvore TF
- `robis_navigation`: consome `/scan`, `/odom`, `/tf` e `/cmd_vel`
- `robis_bringup`: troca a origem dos dados entre `sim` e `real`, mantendo o mesmo stack acima

## Launch files principais

- `simulation_full.launch.py`: simulacao completa + stack opcional (`none`, `slam`, `localization`, `navigation`)
- `real_full.launch.py`: bringup do robo real + stack opcional (`none`, `slam`, `localization`, `navigation`)
- `ball_follow_demo.launch.py`: Gazebo + câmera simulada + seguidor de bola
- `ball_follow_real.launch.py`: robo real + webcam + seguidor de bola

## Dependencias opcionais para o modo real

O modo `real` espera estes pacotes instalados no ambiente:

- `robis_odrive`
- `robis_ydlidar`
- `robis_vision` (opcional, se a camera USB fizer parte do bringup)

## Exemplos


### Robo real apenas com hardware + TF

```bash
ros2 launch robis_bringup real_full.launch.py stack:=none
```

### Robo real com SLAM

```bash
ros2 launch robis_bringup real_full.launch.py stack:=slam
```

### Robo real com navegacao

```bash
ros2 launch robis_bringup real_full.launch.py \
  stack:=navigation \
  map:=~/robis_ws/src/robis_navigation/maps/lab_map.yaml
```

## Observacoes

- O mesmo `robis_navigation` e usado nos dois modos.
- O modo `real` nasce com camera desativada por padrao, porque a webcam costuma precisar de ajuste fino de dispositivo, formato e exposicao.
- Os parametros do ODrive neste pacote foram alinhados ao `robis_description`, que hoje e a fonte canonica do robo.

## Tópicos e parâmetros

Os modos real e simulado usam as mesmas interfaces: `/cmd_vel`, `/odom`, `/scan`,
`/camera/image_raw`, `/tf` e `/tf_static`. O argumento `namespace` da simulação
é opcional e pode ser usado para executar múltiplos robôs.

Os principais argumentos de `real_full.launch.py` são `stack`, `enable_base`,
`enable_lidar`, `enable_camera`, `map`, `nav_params_file` e `slam_params_file`.

## Troubleshooting

- Se o RViz informar que não encontra uma transformação, confirme `ros2 run tf2_tools view_frames` e verifique a cadeia `map → odom → base_footprint`.
- Se o robô não se mover, confirme se há somente um consumidor de `/cmd_vel` e se o ODrive está habilitado.
- O lidar real depende de `robis_ydlidar`, compilado na imagem do robô real.
