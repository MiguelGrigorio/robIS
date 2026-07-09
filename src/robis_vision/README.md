# robis_vision

Pacote de camera e comportamentos simples por visao para o RobIS D2X.

## Camera USB

```bash
ros2 launch robis_vision usb_camera.launch.py
```

Por padrao publica `sensor_msgs/Image` em `/image_raw`, seguindo o pipeline usado no
`camera.py` da raiz.

## Seguidor de bola

```bash
ros2 launch robis_vision ball_follower.launch.py \
  params_file:=/robis_ws/src/robis_vision/config/ball_follower.yaml
```

O node detecta uma bola laranja por HSV, centraliza a bola na imagem e aproxima ou
afasta ate manter o raio alvo. Ajuste os limites HSV e ganhos em `config/`.

Na simulacao use `ball_follower_demo.yaml`, que assina `/demo/camera/image_raw` e
publica `/demo/cmd_vel`.
