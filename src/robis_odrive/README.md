# robis_odrive

Driver dos motores e da odometria.

O perfil básico recebe `/cmd_vel`, publica `/odom` e a transformação
`odom -> base_footprint`. A configuração usada no robô fica em
`robis_bringup/config/odrive_real.yaml`.

A API ODrive é instalada no build da imagem; não precisa ser instalada no host.
A polaridade de avanço foi preservada. O comando angular segue a convenção
ROS quando os lados estão corretamente identificados. A matemática está em
`drive_math.py`; o ciclo de controle e os prazos ficam em `drive_controller.py`.
`odrive_interface.py` acessa a placa, `drive_safety.py` confere o perfil e
supervisiona a telemetria, e `odrive_node.py` cuida das mensagens e serviços ROS.

Os parâmetros de conexão, calibração, engate e publicação são respeitados.
O perfil físico conecta sem engatar os motores nem calibrar automaticamente.
Um `/cmd_vel` válido e não nulo pode engatar os motores. O serviço
`/odrive_node/engage` também permite engate manual. Erros encontrados na
conexão são reportados sem limpar erros nem reiniciar automaticamente a placa.
O driver para ao perder comandos por 0,5 segundo e libera os motores após
confirmar parada por posição/referência, ou após 2 segundos ocioso. Os prazos
são configuráveis. A partida confere serial, firmware e perfil persistido;
o watchdog da placa fica habilitado durante o controle ativo. Corrente,
temperatura, erros, tensão e resíduos de percurso são supervisionados.

O perfil físico usa odometria `encoder`, com unidades explícitas.
`legacy` continua disponível para comparação histórica. Giros ficam bloqueados
por `allow_angular_motion=false` até confirmar esquerda/direita no robô.
Distâncias e orientação ainda precisam de comparação física externa.
`motor_signs` registra o sinal de avanço por eixo físico, independentemente
da seleção `axis_for_right`.

Para inspecionar a placa sem comandos de escrita, dentro da imagem:

```bash
ros2 run robis_odrive inspect_odrive
```

Há um modo de driver simulado, mas ele não representa a física do Gazebo.
O simulador usa as mesmas polaridades e unidades da API 0.5.x, com encoders
fracionários e sem truncar deslocamentos pequenos. Os testes não conectam à placa.

Use `make test` na raiz para compilar os pacotes e executar os testes
públicos, incluindo mensagens e serviços ROS com o driver simulado.
O alvo adicional `make odrive-check` pertence às ferramentas locais `extras/`.
Veja [o comportamento e os serviços](../../docs/odrive.md).

Veja o [README principal](../../README.md) e o
[estado dos ensaios físicos](../../docs/hardware.md).
