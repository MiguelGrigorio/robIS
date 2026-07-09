# robis_bringup

Camada de integracao do projeto para manter o mesmo stack de localizacao e navegacao entre simulacao e robo real.

## Ideia central

O contrato do projeto fica assim:

- `robis_description`: descreve o robo e sua arvore TF
- `robis_navigation`: consome `/scan`, `/odom`, `/tf` e `/cmd_vel`
- `robis_bringup`: troca a origem dos dados entre `sim` e `real`, mantendo o mesmo stack acima

## Launches principais

- `simulation_full.launch.py`: simulacao completa + stack opcional (`none`, `slam`, `localization`, `navigation`)
- `real_full.launch.py`: bringup do robo real + stack opcional (`none`, `slam`, `localization`, `navigation`)
- `ball_follow_demo.launch.py`: Gazebo + camera simulada + seguidor de bola em `/demo`
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
