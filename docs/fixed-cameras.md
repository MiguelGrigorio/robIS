# Câmeras fixas e memória semântica

Para iniciar o ambiente completo, enviar comandos e executar os casos de
aceitação, veja o [guia de uso da simulação semântica](semantic-simulation.md).

As câmeras do ambiente podem cadastrar objetos antes de o robô procurar por
eles. A implementação atual permite validar esse caminho por gravações,
sem câmera física, sem motores e sem abrir uma janela:

```text
RGB → YOLO → projeção calibrada → associação/fusão entre câmeras
    → tópico ROS → memória semântica → consulta pela ação goto_object
```

O detector recebe apenas imagens. As posições verdadeiras dos objetos do
Gazebo ficam restritas ao avaliador. A projeção usa um ponto 2D e uma altura
de referência por classe; não mede profundidade e não faz triangulação.
O ponto padrão é o centro da caixa. No perfil LabSEA, mesas usam a região
mais espessa da máscara (`anchor_method: mask_incenter`) e cadeiras mantêm
o centro da caixa. Sem uma máscara válida, a observação de mesa é descartada.
Replay e entrada contínua usam a mesma seleção de ponto.
A concordância entre câmeras reduz ambiguidades, mas não
garante a identidade ou a posição de um objeto físico.

## Começar sem o robô

Para o uso público, siga o [perfil LabSEA](semantic-simulation.md): os pesos
já acompanham o repositório e o Compose inicia as CCTVs simuladas.
Os alvos `fixed-cameras-*` abaixo são ferramentas locais de pesquisa em
`extras/`. Seus resultados ficam em `extras/data/` no host, montado como
`/robis_ws/data/` nos containers.

Construa a imagem completa e teste geometria, transporte ROS e persistência:

```bash
make robot-navigation-build
make fixed-cameras-check
```

Esse teste usa os quatro NPZ originais, incluindo os 14 coeficientes de
distorção, e pixels calculados analiticamente. Verifica fusão, movimento com
o mesmo ID, atualização da posição, rejeição de observação antiga e expiração
após recarregar o YAML. Não avalia reconhecimento visual. O resultado fica
em `extras/data/fixed-cameras-check/check.json`; para repetir, preserve/mova essa
pasta antes, pois o verificador recusa uma memória existente.

Na Orin com NVIDIA e a imagem experimental do LabSEA já preparada:

```bash
make fixed-cameras-fixture
make fixed-cameras-replay
```

O primeiro comando renderiza duas capturas sincronizadas das quatro câmeras
no mundo controlado `labsea_semantic`, sem robô ou janela. Usa o mesmo
posicionamento dos NPZ e intrínsecos `nK`, que são conferidos no `CameraInfo`.
Cada conjunto tem exatamente o mesmo timestamp nas quatro vistas. Não usa
o GLB completo como cenário deste ensaio. Exige a imagem
`robis:jazzy-labsea-visual` descrita no [guia LabSEA](labsea.md).

O segundo executa o replay com o modelo genérico da imagem de navegação,
em CPU, e salva JSONL. Ele não publica no ROS por padrão. Reconhecimento
correto não é garantido pelo modelo genérico neste cenário sintético.
`FIXED_CAMERA_RECORDING` e `FIXED_CAMERA_OUTPUT` usam caminhos relativos
internos como `data/fixed-cameras-rgb`; no host, correspondem a
`extras/data/fixed-cameras-rgb`. A saída existente não é sobrescrita.

Para verificar o resultado positivo com o detector adaptado já disponível
na imagem experimental, incluindo recebimento ROS, persistência e expiração:

```bash
make fixed-cameras-gpu-build
make fixed-cameras-memory-check
```

O segundo comando usa os pacotes instalados, sem montar fontes da aplicação,
e exige os três objetos do cenário controlado. Os resultados ficam em
`extras/data/fixed-cameras-memory-check/`; preserve/mova a pasta para repetir.

Para usar outros pesos já disponíveis, execute dentro da imagem apropriada:

```bash
ros2 run robis_semantic_navigation fixed_camera_replay \
  --config /robis_ws/data/fixed-cameras-rgb/config.yaml \
  --input /robis_ws/data/fixed-cameras-rgb/images.jsonl \
  --output /robis_ws/data/fixed-cameras-rgb/adapted.jsonl \
  --model /opt/robis-models/labsea-yolo26s-seg.pt --device 0 --image-size 1280
```

Os pesos adaptados também estão na base GPU pública, construída a partir de
`config/models/labsea-yolo26s-seg.pt`; a imagem CPU física usa seu modelo
genérico separado. O programa recusa pesos ausentes e não baixa modelos.
Nenhum treino novo é feito por esses comandos. A projeção usa o ponto
configurado por classe, incluindo `mask_incenter` para mesas no LabSEA;
a fusão embarcada câmera–LiDAR usa máscaras.

## Formato da gravação

Uma linha JSON por frame, em ordem de timestamp; o índice cresce por câmera:

```json
{"camera_id":"1","frame_index":0,"stamp":100.0,"width":1280,"height":720,"image":"camera_1_0.png"}
```

`image` é relativo ao JSONL. As câmeras de uma captura sincronizada usam o
mesmo `stamp`. A imagem deve ter a resolução calibrada, sem recorte ou
redimensionamento. O detector pode redimensioná-la internamente; suas caixas
devem voltar às coordenadas da imagem original.

`--detections-only` aceita `detections` no lugar da inferência, com
`label`, `score` e `bbox: [x1,y1,x2,y2]`. Serve para testes de lógica e fica
explicitamente identificado na proveniência. Não equivale a detecção RGB.

A saída registra hashes de entrada, configuração, calibrações e pesos,
caixas detectadas, tempos de inferência, objetos aceitos e rejeições.
`--checkpoint arquivo.json` salva IDs e estado do rastreador atomicamente.
Na retomada, a configuração/calibração e o modelo devem coincidir, e a
entrada deve continuar a sequência, sem repetir frames anteriores.

## Missão completa no Gazebo

Na Orin com a imagem `robis:jazzy-fixed-camera-gpu` disponível:

```bash
make fixed-cameras-mission-build
make fixed-cameras-mission-check
```

O verificador gera o mundo controlado com as quatro câmeras, cria um mapa
por LiDAR/SLAM com o robô parado e reinicia com AMCL e memória semântica vazia.
As câmeras do teto cadastram os objetos por RGB. Em seguida, consultas por
cadeira e mesa passam pela ação `/goto_object` e por ações reais do Nav2.
A chegada precisa ser confirmada pelo detector embarcado com máscaras e LiDAR.

As quatro câmeras ficam ativas durante o trajeto, com duas capturas por
segundo de simulação. O prazo dos objetos permanece em três segundos; o
teste verifica se o mesmo registro continua válido durante a missão.
O detector do teto usa confiança mínima de 0,4 neste ensaio; o nó genérico
mantém 0,25 por padrão. Esse ajuste reduz previsões fracas do robô como móvel,
mas não garante cobertura das duas cadeiras nem elimina falsos positivos.
Depois das missões, desliga o processamento do teto e verifica a expiração
recarregando o YAML. Não há aumento artificial do prazo para cobrir o trajeto.

O verificador exige os estados `NAVIGATING_TO_MEMORY`, `RECONFIRMING` e
`PERSISTING`, ausência de `SEARCHING`, deslocamento por odometria e incremento
de `onboard_confirmations` no registro externo escolhido. Esse campo e
`last_onboard_confirmed` registram uma confirmação a bordo; novas observações
do teto não incrementam esse contador.

As saídas ficam em `extras/data/fixed-camera-mission/`, incluindo `report.json`,
mapa, trajetória, amostras da validade da memória, imagens e logs separados.
Para repetir preservando a execução anterior:

```bash
make fixed-cameras-mission-check FIXED_CAMERA_MISSION_RUN=outra-execucao
```

O container recebe apenas GPU, dados e scripts verificadores. Os pacotes da
aplicação vêm compilados da imagem. O teste usa seis CPUs, limite de 8 GiB,
domínio ROS 64 e uma partição Gazebo própria; não mede desempenho da Nano.

## Entrada contínua por tópicos ROS

`fixed_camera_stream` processa `Image` e `CameraInfo` das câmeras sem
reposicionar timestamps, ao contrário do replay. Exemplo dentro da imagem GPU:

```bash
ros2 run robis_semantic_navigation fixed_camera_stream --ros-args \
  -p use_sim_time:=true \
  -p configuration:=/robis_ws/data/fixed-camera-mission/calibrated/config.yaml \
  -p model:=/opt/robis-models/labsea-yolo26s-seg.pt -p "device:='0'"
```

O nó aceita apenas conjuntos completos com timestamp de captura idêntico e
`CameraInfo` compatível com os NPZ. Mantém no máximo quatro instantes
pendentes e processa o conjunto completo mais recente, descartando trabalho
antigo. Perda de uma câmera impede novas fusões e não renova a validade dos
objetos. O tópico `/semantic_navigation/fixed_camera_status` e o parâmetro
opcional `output_directory` permitem observar capturas, descartes, inferência
e posições. O diretório recusa um `observations.jsonl` existente.
O parâmetro `confidence` controla o limiar do detector e fica registrado
na proveniência da execução.

Os tópicos padrão são `/fixed/camera_ID/image_raw` e
`/fixed/camera_ID/camera_info`; `image_topic` e `camera_info_topic` em cada
câmera da configuração permitem alterá-los. Essa entrada atende câmeras
sincronizadas expostas em ROS. RTSP/ONVIF, relógios de câmeras independentes
e sincronização aproximada ainda precisam de um adaptador específico.

## Preparar imagens físicas

Copie `config/fixed-cameras.example.yaml` mantendo os caminhos dos NPZ
relativos ao arquivo de configuração, ou use caminhos absolutos. Preencha:

- `environment_id` e `map_checksum`: identificam o ambiente e o SHA-256 da
  imagem do mapa de navegação, por exemplo `map.pgm`.
- `map_from_world`: transformação plana medida do referencial dos NPZ/GLB
  para `map`, em metros e radianos. Não assumir identidade no ambiente real.
- `planes`: altura e incerteza da superfície representada pelo ponto de
  projeção de cada classe. Não é necessariamente a altura total do objeto.
  `anchor_method` seleciona `bbox_center` (padrão) ou `mask_incenter`;
  a segunda opção exige segmentação e uma região válida da máscara.
- `image_space`: `raw` usa `K` e todos os 14 coeficientes; `undistorted` usa
  `nK` e distorção zero, com a imagem completa correspondente.

O template recusa campos medidos ausentes. O gerador sintético preenche
valores exclusivos do cenário de teste; eles não são calibração do ambiente
real. A posição das câmeras no GLB não fornece o alinhamento com o mapa SLAM.
As calibrações fixas também não substituem a da câmera USB do robô.

O limite `maximum_sensitivity_m` rejeita projeções muito sensíveis à
incerteza em pixels/altura. É uma análise de perturbação, não uma covariância
estatística calibrada. Raios acima do horizonte, fora da imagem ou com
interseção inválida são rejeitados.

## Publicação e validade

`--publish-ros` publica `robis_interfaces/ExternalObjects` em
`/semantic_navigation/external_objects`, no ritmo registrado. O replay
reposiciona o início da gravação no relógio ROS atual, preservando os
intervalos e o instante de cada observação individual. É um ensaio de replay,
não uma maneira de tornar capturas físicas antigas recentes.
Para uma simulação usando `/clock`, adicione `--use-sim-time`. Produtor e
servidor precisam usar o mesmo domínio ROS e relógio.

No servidor, habilite `enable_external_memory:=true` e informe
`external_environment_id` e `map_checksum`. Esses argumentos existem nos
launches `semantic_real`, `semantic_simulation` e `semantic_navigation`.
No Compose físico, use `ROBIS_EXTERNAL_MEMORY=true`,
`ROBIS_EXTERNAL_ENVIRONMENT=...` e `ROBIS_MAP_CHECKSUM=...`.
Os limites são `external_max_age_sec` e `external_max_sensitivity_m`
(variáveis Compose `ROBIS_EXTERNAL_MAX_AGE_SEC` e
`ROBIS_EXTERNAL_MAX_SENSITIVITY_M`). A entrada permanece desligada por padrão.

O servidor valida ambiente, checksum, frame, timestamps, confiança,
coordenadas e sensibilidade. Ele é o único escritor do YAML. O processo das
câmeras publica ROS e não edita o arquivo da memória diretamente.

O template pede duas câmeras dentro de uma janela de 0,2 s e pelo menos
duas observações. A associação é geométrica, por classe e proximidade, com
no máximo uma detecção de cada frame por objeto. Uma mudança dentro do raio
de associação atualiza a posição do mesmo ID; movimentos maiores, oclusões
e objetos próximos podem gerar outro ID ou trocar associações. Não há
reidentificação por aparência.

O prazo padrão é três segundos desde a última captura aceita. Uma revisão
de outra câmera pode refinar a mesma captura, sem renovar esse prazo. Frames
duplicados/fora de ordem não renovam objetos. O servidor limita a validade
ao seu próprio máximo e verifica expiração pelos relógios ROS e de sistema;
vence o primeiro limite. Isso é conservador quando a simulação pausa ou
roda mais devagar. Registros expirados ficam no YAML para auditoria, mas
não são escolhidos como destino, inclusive após reiniciar.

A memória externa permite pular a primeira busca quando há um registro
válido. A chegada continua exigindo percepção a bordo e confirmação visual;
nenhuma observação externa simula uma confirmação da câmera do robô.
Uma missão iniciada por um registro externo válido acompanha o mesmo ID
e recalcula a aproximação quando seu deslocamento acumulado atinge o limite
configurado. Veja o [primeiro cenário de objetos móveis](moving-objects.md)
para os parâmetros e limites da associação geométrica.

Quando a chegada falha na reconfirmação, o servidor pode marcar uma instância
externa como `onboard_rejected`. Novos frames do mesmo ID externo não a
reativam; uma confirmação embarcada pode recuperá-la, ou uma nova identidade
do rastreador pode ser admitida. A quarentena é persistida no YAML.

## Validação e próximos dados

O resumo da aplicação atual e da repetição dos casos estáticos e móveis
está na [entrega com simulação](initial-release.md).

O relatório do ensaio (acervo local: `extras/validation/fixed-cameras-2026-10-15.md`) separa os
testes analíticos dos testes RGB e registra os limites observados.
O ensaio de missão completa (acervo local: `extras/validation/fixed-camera-mission-2026-10-18.md`)
registra navegação, confirmação embarcada, validade durante o trajeto e o
falso positivo temporário observado com o robô presente.
`extras/scripts/check_fixed_camera_rgb_memory.py` verifica o caso controlado com
detector adaptado, ROS e recarga da memória; requer a imagem GPU compilada.

Ainda faltam os streams físicos, alinhamento `map` ↔ calibração, alturas
justificadas e avaliação com imagens reais. A entrada contínua ROS está
disponível para imagens sincronizadas; o adaptador dos endereços/protocolos
e relógios das câmeras físicas continua pendente.
