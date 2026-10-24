# robIS

Software do robô robIS em **ROS 2 Jazzy**, com os pacotes em `src/`.
Robô físico, desenvolvimento e simulação usam a mesma distribuição.
Execute os comandos abaixo na raiz do repositório. `make help` mostra os atalhos.

A versão inicial prioriza a simulação: sensores, navegação e memória semântica
por câmeras fixas. O pacote passou pela verificação de publicação em outubro/2026.
Veja [escopo, resultados e limites](docs/initial-release.md).

## Desenvolver e simular

Use Linux com Docker Engine, Compose v2 e Make. ROS e Gazebo ficam no
container; a simulação básica renderiza por CPU e dispensa hardware do robô:

```bash
make sim-up
make sim-logs
# Depois de aparecer Entity creation successful nos logs:
make sim-check
```

`make test` constrói e testa os pacotes em um container isolado, sem hardware.
`make sim-shell` abre um terminal ROS com o código compartilhado.

Depois de editar os pacotes, reinicie o nó afetado. Para C++, arquivos novos
ou alterações de instalação, execute `make sim-build`.
`make sim-down` encerra o ambiente. Veja o [guia de simulação](docs/simulation.md).

Para desenvolver o perfil básico com código compartilhado:

```bash
make dev-up
make dev-shell
make dev-build
```

`make down` encerra os ambientes de desenvolvimento. Os containers de
produção levam código compilado; os de desenvolvimento compartilham `src/`.

## LabSEA e navegação semântica

O perfil GPU preparado para a Orin mantém Gazebo, câmera embarcada,
LiDAR, câmeras do teto, percepção, memória e Nav2.

Na primeira instalação, prepare a base com
`make sim-semantic-base-build`. O checkpoint de segmentação LabSEA já está
em `config/models/`, com hash e manifesto. Depois:

```bash
make sim-semantic-build
make sim-semantic-up
make sim-semantic-shell
```

Dentro do container:

```bash
ros2 run robis_semantic_navigation goto mesa \
  --environment labsea_fixed_fixture --timeout 120
```

Use `make sim-semantic-stop` para cancelar uma missão e
`make sim-semantic-down` para encerrar a simulação.
Esse perfil precisa de GPU NVIDIA e do runtime descrito no
[guia de uso](docs/semantic-simulation.md).

Para observar, conecte o Foxglove a `ws://localhost:8765` e importe
[foxglove/robis.json](foxglove/robis.json). O layout mostra robô, LiDAR,
câmera, mapa e navegação. A aba Controle manual usa o painel Teleop quando
habilitado no bridge; veja [conexão e controle](foxglove/README.md).

Os modelos atualizados do robIS e do LabSEA estão integrados aos respectivos
pacotes. A revisão de outubro/2026 passou nos ensaios do cenário controlado,
incluindo percepção, navegação e objetos deslocados. Veja os resultados e
limites no [guia da simulação semântica](docs/semantic-simulation.md).

## Rodar no robô

Alvo atual: **Jetson Nano 4 GB, JetPack 4.6.6 e L4T 32.7.6**.
A imagem contém os pacotes compilados, o SDK do YDLidar e a API do ODrive.
A inicialização usa essa imagem pronta, sem compilação ou volume de código.

No computador com Docker e um builder ARM64:

```bash
make robot-build
make deploy
```

O destino padrão é `jetson@10.10.1.220`. O SSH usa a chave configurada ou
solicita a senha no terminal. Para outro destino: `make deploy JETSON=usuario@host`.
O deploy transfere e carrega a imagem; a inicialização é feita na Nano:

```bash
~/robis/run_robot.sh
docker logs -f robis
```

Esse comando inicia o perfil básico. Após parar com `docker stop robis`,
use `docker start robis` para iniciar o mesmo container novamente.
A Nano precisa de Docker, acesso SSH e permissão do usuário para executar Docker.

O perfil básico inicia os drivers de motores, LiDAR, câmera e odometria.
A ODrive conecta sem engatar na partida. Para incluir Nav2,
detector e memória semântica, use `make robot-navigation-build` e
`make deploy-navigation`.
Veja [build e deploy](docs/build.md), [compatibilidade da Nano](docs/jetson.md)
e [configuração do perfil completo](docs/real-navigation.md).
O bringup básico conjunto passou na Nano com a imagem compilada. A navegação
física, os giros e a calibração da câmera continuam pendentes.
Veja o [inventário e estado da montagem em outubro/2026](docs/hardware.md).

## Onde encontrar cada coisa

| Pasta | Conteúdo |
| --- | --- |
| `src/` | Pacotes ROS, parâmetros, modelos e mundos usados em execução |
| `docker/` | Dockerfiles, inicialização e perfis Compose de uso |
| `scripts/` | Build, deploy e inicialização |
| `config/` | Perfis de pacotes, pesos do detector e calibrações das câmeras fixas |
| `foxglove/` | Layout e guia de visualização/controle |
| `docs/` | Guias de instalação e uso |
| `extras/` | Material local de desenvolvimento: ensaios, testes, pesquisa e CAD |

Os YAMLs dos pacotes ROS configuram nós e sensores. Os arquivos de
`docker/compose/` escolhem como iniciar os containers: desenvolvimento,
robô básico, navegação no robô ou simulação semântica. O Makefile seleciona
os arquivos e mantém os caminhos relativos à raiz; não é preciso editar
Compose para usar os comandos acima.

Leitura complementar: [navegação](docs/navigation.md),
[memória semântica](docs/semantic-navigation.md),
[câmeras fixas](docs/fixed-cameras.md) e [ODrive](docs/odrive.md).

O SDK do YDLidar é baixado do repositório oficial durante o build da imagem
do robô, com commit fixo. Não é preciso cloná-lo ou instalá-lo manualmente.
