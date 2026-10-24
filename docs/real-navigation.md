# Navegação semântica no robô físico

O perfil `navigation` reutiliza os mesmos nós de percepção, fusão, Nav2 e
memória usados na simulação. `semantic_real.launch.py` usa os drivers de
`basic_real.launch.py`, relógio real e URDF sem plugins do Gazebo.

O perfil físico mantém `allow_angular_motion=false` enquanto os lados das
rodas não forem identificados. Antes de ensaiar Nav2, confirme `axis_for_right`,
valide um giro supervisionado e então habilite comandos angulares no perfil.
O bloqueio atual solicita parada quando chega um comando de giro.

## Imagem completa

```bash
make robot-navigation-build
make deploy-navigation
```

`docker/Dockerfile.robot --target robot-navigation` compila oito pacotes, incluindo
o driver físico do YDLidar, Nav2 e semântica; Gazebo fica de fora. O SDK,
ODrive, dependências e pesos do detector entram no build. Não há compilação,
download de modelos ou montagem de código na inicialização.

O padrão é **YOLO26n-seg pré-treinado**, CPU, imagem de inferência de 320 pixels,
até 1 Hz e duas threads. A arquitetura de fusão é a mesma do teste visual:
`mask_projected_cluster`, dois pixels de tolerância, compensação temporal por
odometria, dois retornos mínimos e três confirmações. A profundidade de
referência fica desligada. Nav2 usa RPP como na simulação, com um arquivo
de parâmetros físico separado. Limites de aceleração, frenagem e giro ainda
precisam ser ajustados e validados com a dinâmica real.

O modelo adaptado ao LabSEA permanece no experimento sintético. Ele não é
selecionado automaticamente para imagens reais. O desempenho do modelo CPU
na Nano e a qualidade de reconhecimento físico ainda precisam ser medidos.
Este perfil não configura a GPU da Nano; a imagem CUDA da Orin é separada.

## Calibração da câmera do robô

Coloque a calibração ROS YAML da câmera USB em:

```text
data/calibration/robot_camera.yaml
```

No deploy sem Compose, a pasta de dados padrão é `~/robis/data/` na Nano.
Com Compose, é `data/` ao lado dos arquivos do repositório. São dados de
execução persistentes, não código-fonte.

A resolução do YAML deve corresponder à captura (padrão 640×480). O arquivo
deve conter `image_width`, `image_height`, `camera_matrix`,
`distortion_coefficients`, `rectification_matrix`, `projection_matrix` e
`distortion_model`. O driver publica imagem e `CameraInfo` com o mesmo
timestamp e `camera_optical_frame`. A projeção do LiDAR considera a
distorção informada, incluindo o modelo equidistante quando selecionado.

`config/calib/calib_rt1.npz` a `calib_rt4.npz` são das **câmeras fixas do
ambiente**, com resolução 1280×720 e 14 coeficientes de distorção. Elas estão
usadas pelo replay das câmeras fixas e não preenchem a calibração USB.
Veja [os dados dessas câmeras](../config/calib/README.md).

O launch semântico exige um arquivo de calibração existente quando inicia
a câmera física. Para somente capturar imagens sem calibração, use
`basic_real.launch.py`; nesse caso `K[0]=0` continua indicando ausência de
calibração, e o localizador não inventa intrínsecos.

## Mapear e recuperar

Com a imagem transferida e a calibração instalada, a inicialização com
Compose é:

```bash
ROBIS_REAL_MODE=mapping make robot-navigation-up
```

Sem Compose, execute na Nano:

```bash
ROBIS_ROBOT_PROFILE=navigation ROBIS_ROBOT_IMAGE=robis:jazzy-navigation \
  ~/robis/run_robot.sh
```

Iniciar esse perfil conecta a ODrive sem engate inicial, conforme o YAML
físico; um comando não nulo posterior pode habilitar os motores.
O script de deploy apenas transfere a imagem; ele não inicia o hardware.
O perfil nativo gravado na ODrive é conferido sem reprogramar seus ganhos
ou limites. O X4 usa as distâncias do SDK, com `scan_gain=1.0`.
O driver ROS já incorpora watchdog, verificação de perfil e guardas de
telemetria e percurso. O bringup básico e um avanço curto foram ensaiados
na Nano; o perfil completo ainda exige calibração e validação física de
curvas, dinâmica e navegação. Veja o [estado do hardware](hardware.md).

O modo `mapping` inicia SLAM e Nav2; a ação `/goto_object` usa a mesma
busca, aproximação e memória da simulação. Depois de mapear, salve o mapa
na pasta persistente:

```bash
docker compose --project-directory . -f docker/compose/robot.yml -f docker/compose/robot-navigation.yml \
  exec robis /usr/local/bin/robis-entrypoint ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap '{name: {data: /robis_ws/data/navigation/map}}'
```

Crie `data/navigation/` antes de salvar. O serviço gera `map.yaml` e `map.pgm`.
Pare a execução de mapeamento e reinicie com o mapa real salvo:

```bash
make robot-navigation-down
ROBIS_REAL_MODE=navigation make robot-navigation-up
```

Esse modo inicia mapa salvo, AMCL e Nav2, reutilizando a memória em
`data/environments/semantic_map.yaml`. A pose inicial de AMCL precisa
corresponder à posição física; use a inicialização de pose do ROS quando
necessário. O PGM da simulação não substitui o mapa real.

`ROBIS_MAP_CHECKSUM` pode receber o SHA-256 de `map.pgm` para vincular a memória
ao mapa. Para preservar a identificação espacial, use o mesmo mapa e origem
na recuperação. Os arquivos de metadados e memória devem acompanhar o mapa.

## Argumentos do launch

Para execução direta dentro da imagem:

```bash
ros2 launch robis_bringup semantic_real.launch.py \
  mode:=mapping camera_info_url:=/robis_ws/data/calibration/robot_camera.yaml
```

O modo `navigation` exige também `map:=/robis_ws/data/navigation/map.yaml`.
O launch expõe arquivos de parâmetros da câmera, ODrive, LiDAR, geometria,
SLAM, Nav2 e semântica, além de modelo, dispositivo, resolução e frequência
do detector. A imagem contém pesos de segmentação e não requer download no boot.

`detector_backend:=external` desliga apenas o detector local. Nesse caso,
outro processo precisa publicar os tópicos ROS de detecções e máscaras de
instância, com timestamps e frames compatíveis com câmera e LiDAR. Isso é
uma entrada para percepção externa da câmera do robô.

A memória pode receber objetos das câmeras do teto em uma entrada separada,
`/semantic_navigation/external_objects`. Para habilitá-la com Compose,
defina `ROBIS_EXTERNAL_MEMORY=true`, `ROBIS_EXTERNAL_ENVIRONMENT` e
`ROBIS_MAP_CHECKSUM`. No launch direto, use `enable_external_memory:=true`,
`external_environment_id:=...` e `map_checksum:=...`. O checksum deve ser
o SHA-256 do mesmo mapa usado pelo produtor externo.

Essa integração permite consultar objetos já vistos no ambiente, sem uma
primeira busca do robô. A confirmação visual a bordo e as verificações do
Nav2 continuam necessárias para concluir a missão. A entrada está desligada
por padrão. Veja [configuração, validade e limites](fixed-cameras.md).

## Verificação sem hardware

O verificador abaixo pertence às ferramentas locais `extras/` e não
acompanha o clone público.

Depois de construir a imagem ARM64, execute em um host capaz de rodá-la:

```bash
make robot-navigation-check
```

`extras/scripts/check_real_launch.py` exercita os dois modos usando o launch
instalado, sensores sintéticos, RGB vazio e os três drivers físicos
explicitamente desligados. Deve ser executado em container sem rede e sem
montagens de dispositivos físicos. Ele não envia metas de navegação ou
comandos de movimento; observa `/cmd_vel` e exige ausência de comandos não nulos.

Essa verificação cobre grafo ROS, relógio real, SLAM/AMCL, estados ativos do
Nav2, detector CPU, parâmetros de fusão e rejeição de consulta vazia.
Ela complementa os ensaios do Gazebo; não mede odometria, localização,
detecção ou navegação no hardware físico. A execução no kernel L4T da Nano
continua pendente.

Os resultados de cada modo ficam em `extras/data/real-launch-check/`.
O teste limita o container a duas CPUs e 4 GiB de memória; isso não reproduz
a velocidade, a memória do sistema operacional ou o kernel da Nano.
