# Simulação do robIS

O Gazebo substitui os motores, o lidar e a câmera. Não é necessário ter o
robô conectado. Execute os comandos na raiz desta pasta, onde está o Makefile.

O [guia LabSEA](labsea.md) descreve o ambiente importado, a edição das posições
dos móveis e o teste de memória semântica sem janela na Orin.
Para assistir a uma missão sem desktop remoto, veja
[como gravar câmera, mapa e navegação em MP4](labsea.md#gravar-uma-missão-em-vídeo).

```bash
make sim-up
make sim-logs
```

Na primeira vez, o Docker baixa as dependências e o Colcon compila os oito
pacotes do perfil. Aguarde a mensagem `Entity creation successful`. Ctrl+C
sai dos logs. Para verificar o funcionamento básico:

```bash
make sim-check
```

O teste confere sensores e referenciais, avança aproximadamente 12 cm, gira
aproximadamente 0,2 rad e verifica a parada na odometria. Retorna JSON e código
zero em caso de sucesso. Esses valores são comandos curtos de teste, sem
pretensão de medir precisão física. O mundo deve estar em execução, sem pausa.
Para conferir somente sensores e TF, sem movimento:

```bash
docker compose --project-directory . -f docker/compose/development.yml exec -e ROBIS_BUILD_ON_START=false robis_sim \
  /usr/local/bin/robis-entrypoint python3 -m robis_gazebo.check_simulation
```

`make test` constrói a imagem e executa os testes públicos dos pacotes em
outro container, isolado da rede e sem dispositivos. Não exige uma simulação
iniciada nem a pasta local `extras/`. Os testes são executados manualmente.
O teste com sensores e movimento (`make sim-check`) é separado,
pois precisa do Gazebo rodando.

## Abrir a janela

```bash
make sim-gui
```

Esse comando recria o serviço com a interface local. O complemento
`docker/compose/simulation-gui.yml` compartilha
o socket X11 e o arquivo indicado por `XAUTHORITY`, sem liberar globalmente
o servidor gráfico. Funciona também em Wayland quando existe XWayland.
Se `XAUTHORITY` não estiver definido, o padrão é `$HOME/.Xauthority`; é
necessário fornecer um arquivo de autenticação válido para abrir a janela.
O modo sem janela utiliza EGL e `--headless-rendering` para continuar gerando
imagem e lidar. Ambos os modos usam renderização por CPU por padrão.

Para voltar ao modo sem janela: `make sim-up`. Para parar: `make down`.
No Fedora Asahi/Apple M1 deste desenvolvimento, a janela e os sensores
funcionaram com Mesa llvmpipe. Não foi configurado acesso à GPU do host.

Se o Docker foi instalado nesta sessão e aparecer `permission denied` no
socket, saia e entre novamente na sessão após entrar no grupo `docker`.
Enquanto isso, se o usuário já consta nesse grupo, `sg docker -c 'make sim-up'`
e `sg docker -c 'make sim-check'` ativam o grupo apenas para esses comandos.

## Editar o código

`src/` é compartilhado com o container. Os volumes de build, instalação e
logs são exclusivos da simulação. Use `make sim-shell` para abrir o terminal
com ROS e os pacotes carregados. Após mudar C++ ou arquivos instalados,
execute `make sim-build`. Reinicie o serviço para recarregar nós e modelo:

```bash
docker compose --project-directory . -f docker/compose/development.yml restart robis_sim
```

O serviço inicia com `stack:=none`, sem SLAM ou navegação autônoma.
`make sim-slam` e `make sim-nav` iniciam cada modo separadamente; o segundo
precisa de um mapa salvo. O ensaio automatizado `make nav-check` faz parte
das ferramentas locais de pesquisa em `extras/`.
Veja [o teste de navegação](navigation.md) e os arquivos gerados.
O bridge Foxglove inicia junto do serviço. Conecte a `ws://localhost:8765`
e importe o [layout do robIS](../foxglove/README.md). Para desligá-lo,
execute `ROBIS_FOXGLOVE_ENABLED=false make sim-up`.

## Como o modelo está integrado

| Responsabilidade | Local |
|---|---|
| Geometria, malhas, sensores e TFs fixas comuns | `src/robis_description/` |
| Mundos, spawn e bridges entre Gazebo e ROS | `src/robis_gazebo/` |
| Composição da simulação com SLAM/Nav2 opcionais | `src/robis_bringup/launch/simulation_full.launch.py` |
| Drivers dos motores e do lidar físicos | `src/robis_odrive/` e `src/robis_ydlidar/` |
| Seleção dos pacotes e ambiente | `config/`, `scripts/` e Compose |

O mesmo Xacro é usado no robô e no Gazebo. Com `sim_flavor:=gz`, ele acrescenta
sensores e plugins de simulação e usa a origem óptica declarada em
`sensors.camera.optical_origin_x/z`, à frente da lente do CAD atualizado.
O perfil físico mantém `physical.camera_offset_x/z`, pendentes de calibração.
O DiffDrive recebe `/cmd_vel` e publica
`/odom` e `odom → base_footprint`. O robot_state_publisher publica as TFs
do modelo. Os bridges entregam lidar e câmera nos tópicos usados no robô.

Profundidade densa e segmentação são recursos de pesquisa. Sensores e seus
bridges só são ligados com `enable_ground_truth_sensors:=true`, nos ensaios
com oráculo. O perfil com detector visual mantém essa opção desligada.
Não representam sensores instalados no robô.
Publicar rótulos perfeitos do simulador também não valida um detector real.

A identificação dos sensores usa a extensão `gz_frame_id` documentada pelo
[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/).
O SDFormat pode emitir aviso sobre essa extensão, mas o Gazebo a reconhece;
o teste verifica o frame resultante e sua transformação.

## Fidelidade ao robô real

O estado atual é uma **simulação funcional integrada**, ainda sem calibração
que permita afirmar que é um Digital Twin fiel. O raio de roda de 0,0854 m
é compatível com a circunferência de 0,5366 m do driver, e ambos usam distância
entre rodas de 0,36 m. Esses parâmetros do driver ainda estão em seu próprio
YAML e precisam ser mantidos coerentes com o modelo.

Há aproximações explícitas:

- O chassi usa colisão cilíndrica. Os casters têm pivô vertical, garfo e roda
  cilíndrica, com juntas passivas e atrito de contato. Geometria do suporte,
  deslocamento do eixo e amortecimento ainda são aproximações configuráveis.
- Massas, inércias, atrito e ruído são valores do modelo existente, sem
  registros de medição nesta etapa. O DiffDrive não reproduz a eletrônica,
  os atrasos e as proteções da ODrive.
- A câmera simulada usa campo de visão e ruído configurados. A câmera física
  ainda depende de calibração fornecida; resolução igual não garante a mesma
  projeção, exposição ou qualidade de imagem.
- A odometria física padrão usa encoders, sinais por eixo e yaw positivo
  conforme ROS. O fator empírico `100` permanece somente no modo legado.
  A simulação usa a odometria do DiffDrive e não exercita a ODrive física.

O bringup físico básico já teve um ensaio separado, descrito no
[inventário de hardware](hardware.md). Para medir fidelidade, ainda são
necessários registros reais de distâncias, giros, velocidades, TFs e
calibração dos sensores, comparados com os mesmos comandos no simulador.

## Casters passivos

Cada caster tem uma junta contínua de giro vertical (`front_caster_joint` ou
`rear_caster_joint`) e uma junta de rolamento da roda (`*_caster_wheel_joint`).
O eixo da roda fica 15 mm atrás do pivô, criando o deslocamento que permite
autoalinhamento. Nenhuma dessas juntas recebe comando do DiffDrive.

O YAML mantém massas totais, raio, pontos de montagem e folga anteriores.
Os novos parâmetros são largura, deslocamento do eixo, fração de massa do
suporte, amortecimento, atrito dos mancais e atrito de contato. A fração de
massa é uma aproximação e não uma medição.

Para conferir o funcionamento físico no Gazebo:

```bash
make sim-casters-check
```

Esse verificador detalhado pertence ao acervo local `extras/`.
No clone público, `make sim-check` verifica os estados e as TFs dos casters
junto com os sensores e os movimentos curtos.

O teste avança, gira, anda de ré e gira novamente. Confere o deslocamento do
corpo pela posição nativa do Gazebo, inclinação, altura e mudança dos ângulos
de rolamento e giro. Não depende apenas da odometria das rodas motrizes.
O mundo padrão precisa estar executando, com o robô em espaço livre.

A folga existente permite que apenas um caster toque o chão quando o chassi
inclina. Por isso, o teste exige movimento do caster carregado, sem exigir
que uma roda suspensa gire. Isso preserva a hipótese de apoio do modelo atual;
não comprova que o apoio é igual ao do robô real.
