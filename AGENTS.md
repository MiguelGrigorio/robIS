# Guia para agentes — robIS

Este arquivo orienta mudanças em todo o repositório. Leia o [README](README.md)
para o fluxo de uso e [o escopo da entrega](docs/initial-release.md) para entender
o que foi validado. Instruções explícitas do usuário têm precedência; instruções
locais em outro `AGENTS.md` podem detalhar o trabalho na respectiva subpasta.

## Objetivo do projeto

O robIS usa ROS 2 para locomover uma base diferencial e navegar até objetos.
A entrega inicial cobre simulação no LabSEA, percepção visual, memória semântica
alimentada por câmeras fixas e confirmação embarcada na chegada. A integração
física básica existe, mas a navegação no robô ainda exige calibração e ensaios.

O repositório deve ser compreensível para quem está começando: mantenha o caminho
principal curto, comandos executáveis e responsabilidades claras. Evite acrescentar
abstrações, serviços, variantes de configuração ou atalhos sem uma necessidade
concreta. Prefira estender o fluxo existente.

Repositório público: <https://github.com/MiguelGrigorio/robIS>.
O nome da pasta do clone pode ser diferente; nunca codifique seu caminho absoluto.

## Onde trabalhar

| Local | Responsabilidade |
| --- | --- |
| `src/robis_bringup/` | Composição dos launches, parâmetros físicos e Foxglove |
| `src/robis_description/` | Xacro, geometria, articulações, TFs e visual do robô |
| `src/robis_gazebo/` | Mundos, LabSEA, sensores simulados, bridge e execução da simulação |
| `src/robis_interfaces/` | Actions, serviços e mensagens compartilhadas |
| `src/robis_navigation/` | SLAM Toolbox, AMCL, Nav2 e mapas |
| `src/robis_odrive/` | Driver, cinemática, odometria, watchdog e proteções da tração |
| `src/robis_vision/` | Captura USB, detecção, máscaras, calibração e preview JPEG |
| `src/robis_semantic_navigation/` | Localização de objetos, CCTVs, memória e missões por objeto |
| `docker/` | Imagens, entrypoint e perfis Compose de operação |
| `scripts/` | Build, deploy, inicialização e verificações públicas |
| `config/` | Listas de pacotes, calibrações fixas e checkpoint do detector |
| `foxglove/` | Layout importável e instruções de observação/controle |
| `docs/` | Guias públicos de instalação, uso, arquitetura e limites |
| `extras/` | Acervo local ignorado pelo Git: pesquisa, campanhas, CAD, logs e relatórios |
| `data/` | Dados gerados em execução: mapas, memória e sessões; ignorados pelo Git |

Parâmetros de um nó pertencem ao pacote correspondente. Configurações de execução
dos containers ficam em `docker/compose/`. Não espalhe Compose e Dockerfiles pela
raiz. Scripts contêm a implementação; o Makefile oferece atalhos curtos.

Um clone público deve funcionar sem `extras/`. O `-include extras/Makefile` é
opcional. Há ferramentas e configurações históricas nos pacotes; sua presença
não as torna o fluxo recomendado. Siga os launches públicos e os guias atuais.
Não mova módulos importados pelo runtime para `extras/` apenas pelo nome do arquivo.

## Ambientes e imagens

- Use **ROS 2 Jazzy / Ubuntu 24.04** em todos os perfis. A simulação usa
  **Gazebo Harmonic**, via `ros_gz`; não introduza uma segunda distribuição ROS
  ou dependências de Gazebo Classic sem uma decisão explícita de arquitetura.
- Produção leva os pacotes compilados e as dependências na imagem. Não exige
  checkout, montagem de `src/`, download de pesos ou compilação na inicialização.
- Desenvolvimento compartilha código pelo Compose, com build/install/log próprios
  por perfil. Não misture instalações de conjuntos diferentes de pacotes.
- A simulação semântica também usa aplicação compilada; apenas dados de sessão
  são compartilhados. Alterações de fonte exigem reconstruir essa imagem.
- Execute Make e Compose na raiz. Para Compose direto, preserve
  `--project-directory .` e o arquivo em `docker/compose/`.

Os conjuntos efetivos de pacotes são definidos em `config/packages.*`:

| Perfil | Pacotes | Uso |
| --- | --- | --- |
| `basic` | 6 | Drivers físicos, interfaces, descrição e bringup; sem Nav2/Gazebo |
| `navigation` | 8 | Básico + navegação e semântica; sem Gazebo |
| `simulation` | 8 | Simulação + navegação/semântica; sem driver físico YDLidar |

Ao acrescentar um pacote ou dependência, confira a lista do perfil, `package.xml`,
instalação em CMake/setup.py e as etapas Docker afetadas. A imagem básica deve
continuar pequena e independente de GPU.

A Nano é uma **Jetson Nano 4 GB B01, JetPack 4.6.6 / L4T 32.7.6**.
Seu host antigo não fornece CUDA compatível com o container Ubuntu 24.04.
O perfil físico atual usa CPU e runtime `runc`; não acrescente `--runtime nvidia`
ou atualize o host como solução automática. Consulte [compatibilidade](docs/jetson.md).

O perfil semântico GPU foi ensaiado em **Orin 64 GB / L4T 39.2.1**, com base
PyTorch/CUDA própria. Não presuma que essa base funciona na Nano ou em toda GPU
NVIDIA. Consulte [o perfil semântico](docs/semantic-simulation.md).

O SDK YDLidar é baixado e compilado no build de `docker/Dockerfile.robot`, com
commit fixado. A API Python ODrive também tem versão fixada; sua versão é distinta
do firmware da placa. Não volte a depender de uma cópia manual do SDK nem troque
dependências por `latest` para contornar falhas. Builds limitam paralelismo para
caber nos 4 GB. Detalhes em [build e deploy](docs/build.md).

## Comandos de referência

Use os alvos existentes; `make help` apresenta o caminho principal.

| Tarefa | Comando |
| --- | --- |
| Build e testes públicos, sem hardware/GPU | `make test` |
| Simulação básica por CPU | `make sim-up`, `make sim-logs`, `make sim-shell` |
| Recompilar código compartilhado | `make sim-build` ou `make dev-build` |
| Verificar sensores, avanço, giro e parada no Gazebo | `make sim-check` |
| Encerrar simulação básica | `make sim-down` |
| Desenvolvimento do perfil básico | `make dev-up`, `make dev-shell` |
| SLAM / navegação com mapa existente | `make sim-slam` / `make sim-nav` |
| Janela opcional do Gazebo | `make sim-gui` |
| Preparar dependências e pesos GPU | `make sim-semantic-base-build` |
| Compilar aplicação semântica | `make sim-semantic-build` |
| Iniciar e abrir terminal semântico | `make sim-semantic-up`, `make sim-semantic-shell` |
| Cancelar missão / encerrar perfil semântico | `make sim-semantic-stop` / `make sim-semantic-down` |
| Construir imagem física básica / completa | `make robot-build` / `make robot-navigation-build` |
| Transferir imagem pronta, sem iniciar hardware | `make deploy` / `make deploy-navigation` |

Na simulação básica, espere `Entity creation successful` antes de `sim-check`.
Esse comando produz movimento simulado; o verificador exige o bridge do Gazebo.
`sim-nav` exige mapa válido. Não substitua um mapa ausente por um arquivo vazio.

No terminal do perfil semântico, uma missão de referência é:

```bash
ros2 run robis_semantic_navigation goto mesa \
  --environment labsea_fixed_fixture --timeout 120
```

O início do perfil prepara o cenário e espera uma consulta; não acrescente uma
missão automática à partida. Consulte os guias antes de executar comandos reais.
Não existe workflow GitHub Actions mantido nesta entrega; os testes são manuais.
Não recrie CI nem acrescente ferramentas de automação sem necessidade da tarefa.

## Contratos de integração

### Sensores, relógios e movimento

- Preserve os tópicos comuns: `/cmd_vel` (`geometry_msgs/msg/Twist`), `/odom`,
  `/scan`, `/camera/image_raw`, `/camera/camera_info`, `/tf` e `/tf_static`.
  `/joint_states` é publicado na simulação; a publicação física pela ODrive
  continua pendente. Não fabrique estados para preencher o painel.
- Na simulação, use `/clock` e `use_sim_time` coerentemente. No físico, use tempo
  real. Observações conservam o timestamp da captura; não as torne recentes
  reescrevendo seu horário. Associe câmera/LiDAR com TF do instante observado.
- A cadeia principal é `map → odom → base_footprint → base_link`, com os frames
  dos sensores definidos no Xacro. Evite duas autoridades para o mesmo TF.
- Preserve unidades ROS: metros, segundos, radianos; avanço em `+x`, giro
  positivo para a esquerda em torno de `+z`. Conversões de encoder/voltas da
  ODrive ficam na camada do driver; não compense erros com escalas arbitrárias.
- Verifique compatibilidade de QoS entre produtores e consumidores de sensores.
  Preserve filas limitadas e a política de frames recentes; não resolva perda
  de mensagens aumentando indefinidamente filas ou timeouts.
- No Gazebo, DiffDrive fornece a odometria principal. Não rode o simulador da
  ODrive publicando outra `/odom` simultaneamente.
- A configuração atual usa **NavFn + Regulated Pure Pursuit**. Nav2 publica em
  `/cmd_vel_nav`, e o suavizador encaminha para `/cmd_vel`. Preserve esse caminho
  ao alterar parâmetros, remapeamentos ou launches.
- Teleop do Foxglove usa `/cmd_vel_teleop → /cmd_vel`, com limites, expiração e
  Stop inicial. Não é árbitro entre Nav2 e controle manual; cancele a missão
  antes de dirigir. Observação é o padrão, teleoperação é opcional.

### Imagens e calibração

- Percepção usa imagem raw; Foxglove usa `/camera/image_raw/compressed`.
  Preserve o preview JPEG com último frame, compressão sob demanda e limite
  configurável de 30 FPS de tempo real. Não faça filas crescerem para esconder
  latência nem envie raw pelo layout padrão. O bridge usa loopback por padrão.
- `config/calib/*.npz` pertence às câmeras do teto. Não reutilize esses arquivos
  na C920. Intrínsecos da câmera simulada vêm da configuração do sensor/CameraInfo.
- A C920 física ainda precisa de calibração própria, na resolução de operação,
  e extrínsecas medidas. `K[0]=0` significa câmera não calibrada; não invente
  intrínsecos ou use calibração da internet para declarar o físico validado.

### Percepção e memória semântica

- O fluxo LabSEA atual usa **YOLO26s-seg**, com checkpoint e manifesto em
  `config/models/`. Preserve o hash e a procedência ao trocar pesos. Treino,
  conversão de CAD e datasets não fazem parte da primeira execução pública.
- CCTVs calibradas produzem `ExternalObjects` em
  `/semantic_navigation/external_objects`; a câmera embarcada com LiDAR
  reconfirma a chegada. A ação de missão é `/goto_object`.
- Ground truth/oráculo pode servir a testes e avaliação. Não injete posições
  verdadeiras na percepção RGB para fazer uma missão visual passar. Declare
  explicitamente quando um ensaio usa oráculo.
- `fixed_camera_stream.py` e `fixed_cameras.py` cuidam da entrada/fusão externa;
  `object_localizer.py` associa percepção embarcada e LiDAR; `server.py` coordena
  a missão; `store.py` persiste a memória. Preserve essa divisão.
- O servidor é o único escritor da memória YAML. Mantenha escrita atômica,
  ambiente, checksum de mapa, identidade das instâncias e reconfirmação.
  Não edite o YAML diretamente de um serviço de câmera.
- Preserve validade por captura, rejeição de frames duplicados/fora de ordem,
  expiração e quarentena de hipóteses externas rejeitadas a bordo. Um heartbeat
  não renova uma observação antiga; reiniciar não reativa um registro expirado.
- Ao acompanhar um objeto deslocado, cancele a ação Nav2 antiga antes de
  replanejar. Perda/expiração do alvo deve encerrar o destino antigo; sucesso
  exige aproximação válida e confirmação embarcada, não apenas sucesso do Nav2.
- `projection_sensitivity_m` expressa sensibilidade geométrica, não covariância
  calibrada. Associação geométrica de IDs não equivale a reidentificação visual.

Veja [memória e missões](docs/semantic-navigation.md),
[câmeras fixas](docs/fixed-cameras.md) e [objetos móveis](docs/moving-objects.md).

## Trabalho com o robô físico

Destino padrão: `jetson@10.10.1.220`, também acessível por alias SSH quando
configurado pelo operador. Nunca grave senhas, chaves ou tokens no repositório.
Ter SSH disponível não confirma área livre ou supervisão física.

Para movimento, use autorização e condições físicas confirmadas na sessão:
posição do robô, espaço livre e pessoa junto ao corte de potência. Não repita
pedidos já respondidos enquanto essas condições continuarem válidas. Se a tarefa
for apenas documentação/simulação, não inicie o robô físico para verificar código.
Considere que um `/cmd_vel` válido e não nulo pode engatar a ODrive.

Preserve as proteções atuais:

- Conexão em IDLE, sem calibração/engate automáticos nem movimento na partida.
- Timeout de comando no host, watchdog da placa, limites de corrente/temperatura,
  supervisão da telemetria e tratamento de falha com parada/desarme.
- Perfil persistente da ODrive: não recalibre, grave FLASH, altere firmware ou
  limpe falhas automaticamente para fazer um teste passar.
- `allow_angular_motion: false` até confirmar qual eixo corresponde a cada roda.
  Eixo 0 negativo e eixo 1 positivo produziram avanço; esquerda/direita continua
  pendente. Não deduza esse mapeamento pelo CAD nem libere giros por suposição.
- MPU6050 com I²C instável permanece fora da fusão de navegação. GPS sem fix
  interno não fornece posição válida. Confira o [inventário](docs/hardware.md).

`make deploy` apenas transfere/carrega a imagem e copia o script de início.
Na Nano, `~/robis/run_robot.sh` cria o container; `docker stop robis` para e
`docker start robis` reinicia o existente. O script recusa substituir um
container já criado. Preserve esse comportamento e os dados ao atualizar.
O perfil completo exige calibração USB e configuração de mapa; veja
[navegação física](docs/real-navigation.md) e [ODrive](docs/odrive.md).

## Como implementar e validar

1. Leia o guia e os arquivos do componente afetado. Confira a árvore de trabalho
   antes de editar: podem existir mudanças do usuário e arquivos novos fora do
   índice. Use `rg` para localizar consumidores de parâmetros, tópicos e APIs.
2. Faça a menor mudança completa. Mantenha lógica de geometria, memória e
   controle testável separadamente dos nós ROS. Siga o estilo do módulo existente.
   Não instale ROS, SDKs ou dependências Python globalmente no host para resolver
   problemas que pertencem às imagens.
3. Se alterar interfaces, atualize produtores, consumidores e testes em conjunto.
   Mudanças em entrypoints/recursos exigem revisar `setup.py` ou CMake; uma
   importação a partir de `src/` não prova que o pacote instalado funciona.
4. Valide na medida da mudança, conforme abaixo. Informe o que executou e o que
   ficou sem testar; não use resultados antigos como se fossem da nova revisão.
5. Atualize os guias e exemplos afetados. Confira `git diff --check` e revise o
   diff final, incluindo arquivos novos. Relate resultado e limites concretos.

| Mudança | Validação apropriada |
| --- | --- |
| Texto/documentação | Comandos citados, caminhos e links; não exige iniciar simulação |
| Lógica Python | Testes existentes do módulo e casos novos para comportamento relevante |
| Pacotes, dependências, instalação ou interfaces ROS | `make test`; acrescente build do perfil físico se afetado |
| Xacro, sensores, TF, bridge ou movimento simulado | `make test` e simulação básica com `make sim-check` |
| Percepção, persistência, validade ou missões | Testes do componente; integração semântica na GPU se disponível |
| SDK, Docker ou scripts de deploy | Verificação do caminho alterado e build correspondente; deploy não prova hardware |
| Controle físico | Testes sem hardware primeiro; ensaio supervisionado somente nas condições acima |

`make test` compila oito pacotes e executa a suíte pública em container isolado,
sem rede de execução, GPU ou dispositivos. Ele não executa toda a bateria GPU
nem valida o driver físico do lidar. É possível rodar um teste específico dentro
do ambiente ROS preparado; não presuma ROS instalado no shell do host.
Não escreva testes que apenas reproduzem a implementação nem rode toda a
bateria repetidamente sem alteração ou dúvida que justifique isso.

Em cenários semânticos, confira também ausência de objeto, hipótese falsa,
cancelamento, destino ocupado, expiração/perda das câmeras e objeto deslocado,
quando forem pertinentes à mudança. Um retorno de cancelamento não prova parada:
verifique comandos/odometria sem o avaliador enviar zeros que escondam falhas.

Isole ensaios simultâneos por projeto Compose, domínio ROS, `GZ_PARTITION`, portas
e diretório de saída. Não conecte uma simulação ao domínio do robô real. Preserve
containers e serviços de outras pessoas; encerre apenas os recursos criados para
a tarefa. Não limpe volumes ou sessões para esconder um teste falho.
Os padrões atuais são domínio 27 para robô/desenvolvimento, 28 para simulação
básica e 68 para o Compose semântico; confira as variáveis do perfil antes do uso.

## Evidência, documentação e publicação

- Escreva guias para usuários em português claro. Explique o fluxo recomendado
  antes das alternativas. Não prometa que simulação aprovada garante o físico.
- No README principal, use mês/ano para a referência da entrega. Relatórios de
  ensaio podem ter data/hora exatas para rastreabilidade.
- Coloque novos relatórios extensos, benchmarks e JSONs de validação em
  `extras/validation/`; resuma nos guias públicos o necessário para compreender
  o resultado. Links públicos não devem depender de arquivos privados existirem.
- Para experimentos, registre entradas, cenário, modelo/hash, imagem, parâmetros,
  critério de sucesso, resultado e limitações. Preserve resultados negativos.
  Não sobrescreva evidência histórica para fazê-la parecer atualizada.
- O escopo aceito é um cenário controlado, mapa parcial e pose AMCL conhecida.
  Móveis reposicionados não validam acompanhamento contínuo. Massas, atrito,
  casters e extrínsecas aproximados não estabelecem fidelidade de Digital Twin.
- Preserve mudanças e staging do usuário. Não use `git reset --hard`, `git clean`,
  staging amplo ou descarte de arquivos para obter uma árvore limpa. `git diff`
  sozinho não mostra os arquivos novos; confira também `git status`.
- Não force a inclusão de `extras/`, `data/`, `.env`, caches ou arquivos gerados.
  Não versione credenciais nem caminhos locais de máquinas de teste.
- Commit, push, tag, publicação de imagem e alteração de remotes devem corresponder
  ao pedido do usuário. Antes de publicar, confira adições e remoções completas
  e a independência de um clone sem `extras/`. Mantenha este guia coerente quando
  arquitetura, comandos ou contratos mudarem.
