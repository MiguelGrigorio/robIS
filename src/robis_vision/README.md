# robis_vision

Pacote de câmera e comportamentos simples por visão para o robIS.

## O que este pacote entrega

- `usb_camera.launch.py`: Launch para câmera USB real
- `ball_follower.launch.py`: Launch para seguidor de bola (real ou simulado)
- `config/ball_follower.yaml`: Configuração para uso com câmera física
- `config/ball_follower_demo.yaml`: Configuração para uso em simulação
- `robis_vision/ball_follower.py`: Node do seguidor de bola implementado em Python

## Como usar a câmera USB

```bash
ros2 launch robis_vision usb_camera.launch.py
```

Por padrão publica `sensor_msgs/Image` em `/camera/image_raw`.

## Seguidor de bola

```bash
# Em ambiente real:
ros2 launch robis_vision ball_follower.launch.py \
  params_file:=/robis_ws/src/robis_vision/config/ball_follower.yaml

# Em simulação:
ros2 launch robis_vision ball_follower.launch.py \
  params_file:=/robis_ws/src/robis_vision/config/ball_follower_demo.yaml
```

O node detecta uma bola laranja por HSV, centraliza a bola na imagem e aproxima ou
afasta até manter o raio alvo. Ajuste os limites HSV e ganhos em `config/`.

## Tópicos, parâmetros e troubleshooting

- O seguidor de bola funciona tanto em ambiente real quanto em simulação
- Para ambientes simulados, utilize a configuração `ball_follower_demo.yaml`
- A câmera de simulação publica em `/camera/image_raw` e o controle é feito em `/cmd_vel`
- Os parâmetros HSV podem ser ajustados para diferentes condições de iluminação
- A câmera publica `/camera/image_raw` (`sensor_msgs/msg/Image`).
- O seguidor assina `/camera/image_raw` e publica comandos em `/cmd_vel` (`geometry_msgs/msg/Twist`).
- A imagem de depuração é publicada em `/ball_follower/debug_image` quando habilitada.
- Os principais parâmetros são `image_topic`, `cmd_vel_topic`, limites HSV, `min_area_px`,
  `target_radius_px`, `max_linear_speed`, `max_angular_speed` e `stop_on_lost`.
- Se não houver comandos, confirme que a câmera publica imagens e que a bola está dentro da faixa HSV configurada.

## Como testar em ambientes simulados

Para testar o seguidor de bola em ambiente simulado, você pode usar a nova world `ball_follower_test.world`:

```bash
ros2 launch robis_bringup simulation_full.launch.py world:=/robis_ws/src/robis_gazebo/worlds/ball_follower_test.world stack:=slam
```

Então inicie o seguidor de bola com:
```bash
ros2 launch robis_vision ball_follower.launch.py params_file:=/robis_ws/src/robis_vision/config/ball_follower_demo.yaml
```

## Estrutura do pacote

```text
robis_vision/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── ball_follower.yaml
│   └── ball_follower_demo.yaml
├── launch/
│   ├── usb_camera.launch.py
│   └── ball_follower.launch.py
├── robis_vision/
│   └── ball_follower.py
└── setup.cfg
