# Mapeamento e navegação no Gazebo

O teste usa apenas a simulação. O Gazebo fornece motores e sensores, o
SLAM Toolbox constrói o mapa, o AMCL localiza o robô no mapa salvo e o Nav2
envia velocidades à base. Não é necessário conectar hardware.

## Executar a missão completa

O verificador `nav-check` descrito nesta seção é uma ferramenta local em
`extras/`; não acompanha o clone público. Para uso público, inicie os modos
SLAM/AMCL abaixo ou siga a [simulação semântica](semantic-simulation.md), que
prepara seu mapa e recebe destinos por comandos ROS.
O fluxo de salvar mapas manualmente está no
[guia de mapas](../src/robis_navigation/maps/README.md).

Na raiz do workspace:

```bash
make nav-check
```

O comando constrói a imagem se necessário e recria o mundo duas vezes.
Primeiro percorre nove pontos com SLAM e salva `extras/data/navigation/map.yaml`
e `map.pgm`. Depois reinicia o mundo na posição original, carrega esse mapa
com AMCL e envia três destinos ao Nav2. Os arquivos de uma execução anterior
nessa pasta são substituídos. Não envie comandos manuais durante o teste.

Os pontos ficam em `config/navigation_check.yaml` e foram escolhidos para o
mundo padrão `robis_turtlebot3_like`. O percurso do mapeamento é previamente
definido; não é exploração autônoma. Os destinos de navegação têm caminhos
calculados pelo Nav2, incluindo desvios das paredes internas.

O teste exige sensores, relógio e a cadeia de TF `map → odom → base_footprint`,
nove nós de localização/navegação ativos e respostas das ações. A chegada deve
ficar a até 30 cm do destino tanto no mapa quanto na posição nativa do Gazebo.
O primeiro caminho precisa ser pelo menos 50% maior que a distância direta,
pois existe uma parede entre origem e destino.

Ao final, inicia outro movimento, cancela a ação e verifica a parada pela
odometria recente, sem publicar zeros para produzir essa parada. Também pede
um caminho para fora do mapa e exige uma resposta de falha do planejador.
Se houver falha ou interrupção, tenta cancelar a ação e publica parada apenas
após confirmar que está conectado ao bridge do Gazebo.

Relatórios e logs:

```text
extras/data/navigation/map-report.json       Mapa e percurso do SLAM
extras/data/navigation/navigate-report.json  Destinos, caminhos, trajetórias e erros
extras/log/navigation-mapping.log            Progresso do mapeamento
extras/log/navigation-mapping-stack.log      Nós ROS durante o mapeamento
extras/log/navigation-mission.log            Progresso da navegação
extras/log/navigation-stack.log              Nós ROS durante a navegação
extras/data/navigation/environment.json      Versões, arquitetura e hashes das entradas
extras/data/navigation/inputs/               Cópias das configurações, modelo e mundo
```

O comando retorna zero apenas quando as duas etapas passam. Ao terminar,
o serviço continua com AMCL e Nav2 iniciados. `make down` encerra a simulação.
O tempo total depende do computador e da renderização dos sensores por CPU;
os limites de espera são de tempo de parede, mesmo com `/clock` mais lento.
No ensaio ARM64/Apple M1, as duas fases somaram aproximadamente seis minutos,
além do build e reinícios.
O relatório do ensaio (acervo local: `extras/validation/navigation-2026-10-12.md`) descreve o método,
as observações, os resultados e os limites para uso em relatório acadêmico.

## Iniciar cada modo

Os comandos desta seção usam a pasta pública de dados `data/` na raiz.
O ensaio privado acima monta `extras/data/` no mesmo caminho interno do
container; seu mapa não é transferido automaticamente para `data/`.

```bash
make sim-slam  # SLAM, para construir outro mapa
make sim-nav   # AMCL e Nav2, usando o mapa salvo
```

Para carregar outro mapa, o caminho deve existir dentro do container:

```bash
ROBIS_MAP_FILE=/robis_ws/data/meu_mapa/map.yaml make sim-nav
```

A configuração do AMCL presume início em `(0, 0, 0)` no mapa. Um mapa com outra
origem requer ajustar a pose inicial ou enviar `/initialpose`. Esse teste usa
o reinício na origem conhecida; não testa localização global sem pose inicial.

Para abrir o Gazebo com navegação:

```bash
ROBIS_SIM_STACK=navigation make sim-gui
```

Isso reinicia o mundo e mantém o mapa salvo. No Foxglove, a aba **Navegação**
do layout `foxglove/robis.json` mostra mapa, lidar, caminho `/plan`, pose AMCL
e o destino do teste. Os costmaps podem ser habilitados no painel 3D.

## Configuração do Nav2

O perfil da simulação usa `nav2_params_demo.yaml`. O costmap global contém
o mapa estático, observações do lidar e inflação; o local contém lidar e
inflação em uma janela móvel. O raio de 0,242 m corresponde à colisão
cilíndrica do modelo, com a margem adicional padrão do costmap.

O NavFn calcula o caminho e o **Regulated Pure Pursuit** o acompanha, reduzindo
velocidade em curvas e perto de obstáculos, e verificando colisões no costmap.
O suavizador entrega o comando final ao DiffDrive. O perfil físico
`nav2_params.yaml` também usa NavFn e Regulated Pure Pursuit, com configuração
separada. Sua dinâmica e seus limites ainda precisam de ajuste e validação
no robô. A navegação não entra na imagem básica da Nano.

Essa missão valida SLAM, persistência do mapa, localização após reinício e
navegação entre destinos geométricos. Não testa obstáculos móveis, pessoas,
localização após se perder, percepção de objetos ou aproximação semântica.
O perfil de simulação atual prioriza deslocamento para frente e rotação no
local; comportamento de ré perto de objetos precisa ser avaliado na etapa
semântica. Não comprova precisão ou segurança do robô físico.

Referências: [Regulated Pure Pursuit no Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
e [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox).
