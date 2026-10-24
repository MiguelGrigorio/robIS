# Mapas de navegação

Esta pasta contém os exemplos `blank_map.yaml`/`.pgm` e `lab_map.yaml`/`.pgm`.
O mapa vazio é um placeholder. Para navegar em um ambiente, use um mapa
correspondente ao mundo ou espaço físico em que o robô está localizado.

Os mapas novos ficam em `/robis_ws/data/navigation/` dentro do container,
compartilhado com `data/navigation/` no host. Isso preserva o resultado após
recriar o container e mantém os dados gerados fora de `src/`.

## Salvar um mapa da simulação básica

Na raiz do repositório, inicie SLAM e abra um terminal ROS:

```bash
make sim-slam
make sim-shell
```

Após mover o robô simulado para observar o ambiente, salve dentro do container:

```bash
mkdir -p /robis_ws/data/navigation
ros2 run nav2_map_server map_saver_cli -f /robis_ws/data/navigation/map
```

Esse comando gera `map.yaml` e `map.pgm`. Para usar outro nome, ajuste também
o caminho informado ao iniciar a navegação. No terminal do host:

```bash
make sim-nav
```

O bringup inicia mapa salvo, AMCL e Nav2, reiniciando o mundo na pose inicial
conhecida. O padrão é `/robis_ws/data/navigation/map.yaml`.
Veja [mapeamento e navegação](../../../docs/navigation.md).

Ao iniciar os launches separadamente, `localization.launch.py` recebe o
argumento `map`. `navigation.launch.py` inicia os servidores Nav2 e depende
da localização já disponível.

O perfil [LabSEA semântico](../../../docs/semantic-simulation.md) prepara
seu próprio mapa parcial e salva os arquivos em `data/semantic-manual/`.
