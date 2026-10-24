# robis_bringup

Reúne os launches que iniciam os componentes do robô.

| Launch | Finalidade |
| --- | --- |
| `basic_real.launch.py` | Drivers físicos, odometria e descrição do robô |
| `real_full.launch.py` | Perfil físico com `stack:=slam`, `mapping`, `localization` ou `navigation`; o padrão é `none` |
| `semantic_real.launch.py` | Perfil físico com percepção e memória, em modo `mapping` ou `navigation` |
| `simulation_full.launch.py` | Gazebo com SLAM/localização/Nav2 selecionados por `stack` |
| `semantic_simulation.launch.py` | Composição da percepção e navegação semântica no Gazebo |
| `foxglove.launch.py` | Bridge, preview JPEG e teleoperação opcional |

O perfil atual da Nano usa `basic_real.launch.py`: motores, lidar, câmera e
modelo do robô. Ele não depende de Nav2, Gazebo ou navegação semântica.

As configurações ficam em `config/`: `odrive_real.yaml`,
`ydlidar_real.yaml` e `usb_cam_basic.yaml`.
Os argumentos `enable_base`, `enable_lidar` e `enable_camera` permitem
selecionar quais componentes iniciar.
O argumento `enable_foxglove` habilita a observação opcional no perfil básico.
`foxglove.launch.py` também permite iniciar o bridge separadamente.
`enable_teleop:=true` habilita o painel Teleop pelo encaminhador
`/cmd_vel_teleop → /cmd_vel`, com limites e expiração dos comandos. O padrão
continua sendo somente observação. Veja o [guia do Foxglove](../../foxglove/README.md).

`real_full.launch.py` reutiliza esses drivers e acrescenta SLAM ou AMCL e
Nav2. `semantic_real.launch.py` também inicia detector, fusão e memória;
esse perfil usa a imagem `robot-navigation` e exige calibração da câmera USB.
Veja [o perfil real completo](../../docs/real-navigation.md).

Na raiz do repositório, `make sim-up` usa `simulation_full.launch.py` com
`stack:=none`. O perfil LabSEA com CCTVs e detector visual é iniciado por
`make sim-semantic-up`, após construir suas imagens conforme o
[guia de simulação semântica](../../docs/semantic-simulation.md).

Comece pelo [README principal](../../README.md) e pelo [guia de build](../../docs/build.md).
