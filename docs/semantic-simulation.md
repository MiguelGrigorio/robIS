# Usar e verificar a simulação semântica

Este perfil mantém Gazebo, LiDAR, câmera embarcada, quatro câmeras do teto,
detectores, memória, AMCL e Nav2 rodando para comandos manuais. Usa ROS 2
Jazzy e o YOLO adaptado ao cenário controlado LabSEA, sem treino na inicialização.
É um perfil GPU para a Orin preparada nos ensaios; não é a imagem da Nano.

## Iniciar

Na primeira instalação na Orin, confira o checkpoint de segmentação incluído
no repositório (classes `chair` e `dining table`) e construa a base GPU:

```bash
(cd config/models && sha256sum -c SHA256SUMS)
make sim-semantic-base-build
```

O [manifesto dos pesos](../config/models/labsea-yolo26s-seg.json) registra
hash, classes e origem. Para usar outro checkpoint compatível, informe
`SEMANTIC_WEIGHTS=/caminho/best.pt`. Não há treinamento ao construir ou iniciar.

Isso constrói ROS/Gazebo, PyTorch CUDA e incorpora os pesos. O índice padrão
é CUDA 13.0, usado na Orin L4T 39.2.1; não se aplica à GPU da Nano.
O comando dispensa as antigas imagens de experimentos. Confira o
[índice oficial de pacotes PyTorch](https://download.pytorch.org/whl/cu130/).
`SEMANTIC_TORCH_INDEX` permite selecionar outro runtime compatível.

Na raiz do repositório na Orin, com `robis:jazzy-semantic-base` disponível:

```bash
make sim-semantic-build
make sim-semantic-up
```

O primeiro comando incorpora os pacotes e scripts atuais à imagem. O segundo
usa Compose e espera a simulação ficar pronta; não envia metas. Na preparação,
o robô fica parado, SLAM salva um mapa parcial e o mundo reinicia com AMCL.
Depois, as câmeras do teto preenchem uma memória inicialmente vazia.
O ambiente permanece rodando até `make sim-semantic-down`.

Para acompanhar a inicialização: `make sim-semantic-logs`. Ctrl+C nesse comando
encerra somente a visualização dos logs. O container recebe dados e GPU;
não monta código, dispositivos USB/serial nem compila ao iniciar.
Depois de alterar os pacotes desse perfil, encerre, reconstrua a imagem e
inicie novamente. O ambiente de desenvolvimento com `src/` compartilhado
continua disponível pelo fluxo `make sim-up` do README.

Cada início gera uma sessão em `data/semantic-manual/session-.../`, preservando
as anteriores. `data/semantic-manual/current.json` registra a última sessão inicializada; o estado atual vem do Compose.
O mundo e a memória reiniciam em cada sessão; esse perfil não retoma uma
missão que estava ativa antes de desligar.

## Enviar comandos

Abra um terminal com o ambiente ROS configurado:

```bash
make sim-semantic-shell
```

Dentro dele, envie uma consulta por objeto:

```bash
ros2 run robis_semantic_navigation goto cadeira \
  --environment labsea_fixed_fixture --timeout 120

ros2 run robis_semantic_navigation goto mesa \
  --environment labsea_fixed_fixture --timeout 120
```

O servidor escolhe uma instância válida da memória e uma aproximação
planejável, navega e exige confirmação pela própria câmera com LiDAR.
Se a instância vem das CCTVs, mudanças de posição atualizam a aproximação;
uma posição expirada cancela o destino antigo. Os detalhes e o ensaio estão
em [objetos móveis](moving-objects.md).
Se não houver memória válida, pode procurar o objeto pela percepção embarcada.
O terminal mostra andamento, resultado e métricas. O ambiente identifica o
grupo de objetos no YAML e deve corresponder ao usado pelas câmeras.

Para ir diretamente a uma coordenada livre do mapa:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

X e Y são metros no referencial `map`, não deslocamentos relativos à posição
atual. A orientação do exemplo aponta para +X. O mapa é parcial: escolha
um destino dentro da região livre observada. Envie uma missão de cada vez;
não misture metas geométricas e semânticas simultâneas.
Neste perfil, a tolerância do NavFn para deslocar uma meta pedida é zero;
o serviço semântico já escolhe candidatos de aproximação livres. A tolerância
de chegada do controlador permanece em 0,20 m. Uma coordenada ocupada não
é silenciosamente substituída por outra a até 0,5 m dela.

O perfil de simulação permite até 500 ms para a confirmação interna das
ações do Nav2. Esse prazo de comunicação evita abortos espúrios sob carga;
não altera o timeout das missões, a tolerância de chegada nem o TTL de 3 s
das observações das câmeras.

Em outro terminal no host, cancele as ações de navegação:

```bash
make sim-semantic-stop
```

Ou, no terminal ROS, execute `ros2 run robis_semantic_navigation stop`.
Esse cliente solicita cancelamento das ações semântica, de navegação e de
giro. Ele não encerra a simulação. Encerrar apenas um terminal cliente de
uma ação não deve ser usado como confirmação de parada; use o comando de cancelamento.

Para desligar o ambiente:

```bash
make sim-semantic-down
```

## Observar no Foxglove

O bridge escuta `127.0.0.1:8765` na Orin. Para acessar de outro computador:

```bash
ssh -L 8765:127.0.0.1:8765 usuario@orin
```

Conecte a `ws://localhost:8765` e importe o layout `foxglove/robis.json`.
Mapa, câmera, laser, TF e caminho vêm dos tópicos ROS da simulação.
A câmera do layout usa `/camera/image_raw/compressed`, com JPEG e limite
de 30 fps, preservando o fluxo raw utilizado pelo detector. Depois de
atualizar os pacotes, reconstrua a imagem e reimporte o layout.
Veja o [guia do Foxglove](../foxglove/README.md#imagem-e-tráfego-de-rede).
`/semantic_navigation/fixed_camera_status` mostra o estado da memória do teto,
incluindo capturas descartadas e detecções filtradas sobre o próprio robô.

`ROBIS_SEMANTIC_FOXGLOVE_PORT` permite escolher outra porta no host.
O perfil usa domínio ROS 68 e partição Gazebo `robis_semantic_manual`,
separados da bateria de aceitação. O bridge é de observação por padrão:
os comandos documentados acima são os clientes de missão verificados.
O [controle manual opcional](../foxglove/README.md#controle-manual) usa o
painel Teleop e exige `enable_teleop:=true` no launch do bridge.

## Bateria de aceitação

Os executores completos abaixo pertencem ao acervo local de pesquisa em
`extras/`, mantido fora do Git. O clone público permite executar a aplicação,
enviar missões e rodar `make test` e `make sim-check`. Os resultados da bateria
GPU já realizada estão no [registro público da versão inicial](initial-release.md).

```bash
make sim-semantic-check
```

O teste usa outro container, domínio ROS 69 e partição Gazebo própria.
Recusa sobrescrever uma execução existente em `extras/data/semantic-acceptance/`.
Para preservar e repetir: `make sim-semantic-check SEMANTIC_ACCEPTANCE_RUN=outro-nome`.
Evite executar o modo manual e a aceitação ao mesmo tempo na mesma GPU.

São verificados:

1. Cadeira e mesa em três poses iniciais declaradas, com memória vazia em
   cada reinício, navegação real, reconfirmação embarcada e ID preservado.
2. Objeto ausente e término por timeout sem sucesso indevido.
3. Hipótese externa falsa injetada por mensagem ROS, consulta/reconfirmação e
   quarentena após a confirmação falhar, mesmo com novas mensagens da mesma identidade.
4. Cancelamento durante movimento e parada observada pela odometria recente,
   sem o verificador publicar comandos zero para produzir o resultado.
5. Destino dentro de um obstáculo real do Gazebo, recusado pelo planner com
   `GOAL_OCCUPIED`, sem enviar movimento até esse ponto ocupado.
6. Perda das câmeras do teto, expiração da memória e recusa de usar um registro
   expirado como destino; a câmera embarcada continua disponível.

As posições verdadeiras dos móveis são usadas somente na avaliação após
as missões e para contabilizar falsos registros na gravação. A injeção de
uma hipótese falsa é um teste deliberado de falha, separado do reconhecimento RGB.
Não se usa essa hipótese como evidência da qualidade do detector.

## Limites desta etapa

A [conferência para publicação de 23/10/2026](initial-release.md#conferência-do-pacote-em-23102026)
reconstruiu a aplicação atual, verificou o Compose público e repetiu os
11 casos estáticos e os três de móveis deslocados com sucesso na Orin.

Na revisão de **22/10/2026**, passaram 19 testes de modelos/utilitários,
os 11 casos estáticos nas três poses iniciais e os três casos de objetos
móveis. A exportação COLLADA foi corrigida sem alterar vértices, normais
ou UVs, e a origem óptica simulada ficou à frente da lente visual. A câmera
física mantém suas extrínsecas anteriores, pendentes de calibração.

A recepção raw confiável e as filas de um quadro foram verificadas desde a
partida, incluindo o preview JPEG. A inferência das câmeras do teto teve
mediana de aproximadamente **192 ms por imagem**, em 673 imagens 1280×720,
com YOLO a 1280 pixels e PyTorch 2.9.1 CUDA 13.0 na Orin. Isso não é uma
medição de FPS da câmera física nem inclui todo o tempo de navegação.

O registro local da revisão (acervo local: `extras/validation/simulation-fixes-2026-10-22-results.json`)
aponta os relatórios, imagens e logs em `extras/data/simulation-fixes-2026-10-22/`.
Os critérios de chegada, a validade de 3 s das observações e os limites das
missões foram preservados. Os ensaios históricos permanecem no
relatório de aceitação (acervo local: `extras/validation/semantic-acceptance-2026-10-18.md`),
com a execução manual, os 11 casos e as limitações observadas.

O teste cobre móveis parados e trajetos curtos no cenário controlado,
com inicialização conhecida de AMCL. Não comprova exploração completa do
GLB, localização global ou navegação física. A posição das câmeras precisa
ser alinhada ao mapa real antes de usar a mesma ideia com imagens físicas.

A exclusão do próprio robô usa um volume aproximado de 0,6×0,6×0,32 m e
margem horizontal de pose de 0,15 m por lado. A silhueta é projetada com TF
no instante da captura. Se essa TF não estiver disponível, o conjunto espera
ou é descartado ao envelhecer, sem renovar objetos artificialmente.
Uma parte de móvel encoberta pelo robô pode ser descartada junto: isso
reduz cobertura para evitar cadastrar o próprio robô como móvel.
Esses parâmetros precisam acompanhar mudanças no modelo e sua precisão de localização.

Uma instância externa recusada na chegada permanece em quarentena para o
mesmo `source_id`/ID do rastreador, inclusive após recarregar o YAML.
Uma nova identidade pode ser admitida, e uma confirmação embarcada pode
recuperar o registro. Isso não equivale a eliminar todos os falsos positivos.
Para o primeiro ensaio de móveis deslocados, veja [objetos móveis](moving-objects.md).
