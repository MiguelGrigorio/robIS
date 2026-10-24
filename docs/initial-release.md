# Versão inicial com simulação

Esta entrega reúne ROS 2 Jazzy, Gazebo Harmonic, modelos do robIS e LabSEA,
visualização Foxglove e navegação por objetos. O fluxo público pode ser
executado a partir do clone, com os pesos do detector em `config/models/`.
Experimentos, datasets de treino, logs e relatórios extensos ficam em
`extras/`, fora do Git. A preparação local não publica commits, tags ou imagens.

## Começar pelo básico

Pré-requisitos: Linux com Docker Engine, Compose v2 e Make, permissão para
executar Docker e acesso à internet para o primeiro build. ROS e Gazebo
são instalados no container. A renderização básica usa CPU; a velocidade
depende do computador. Não exige câmera, LiDAR ou motores físicos.

```bash
make test
make sim-up
make sim-logs
# Após aparecer Entity creation successful, em outro terminal:
make sim-check
make sim-down
```

`sim-check` valida câmera, CameraInfo, LiDAR, relógio, estados das juntas e TFs;
depois avança, gira e verifica a parada. Para observar, use Foxglove em
`ws://localhost:8765` e importe [o layout](../foxglove/robis.json).
O serviço inicia sem navegação autônoma. Veja [simulação](simulation.md).

## Executar o LabSEA semântico

O perfil completo foi ensaiado na Jetson Orin 64 GB, L4T 39.2.1, com runtime
NVIDIA para Docker. Sua base usa PyTorch 2.9.1 CUDA 13.0. Outros hosts NVIDIA
precisam de um runtime compatível; o perfil básico por CPU continua separado.

```bash
(cd config/models && sha256sum -c SHA256SUMS)
make sim-semantic-base-build
make sim-semantic-build
make sim-semantic-up
make sim-semantic-shell
```

Dentro do container:

```bash
ros2 run robis_semantic_navigation goto mesa \
  --environment labsea_fixed_fixture --timeout 120
```

No host, `make sim-semantic-stop` cancela as ações e
`make sim-semantic-down` encerra o ambiente. O início prepara um mapa parcial
e a memória das CCTVs, aguardando comandos. Cada sessão preserva seus arquivos
em `data/semantic-manual/`. Veja [o guia completo](semantic-simulation.md).

## Evidência registrada

Os resultados de 22/10/2026, preservados no acervo local
`extras/validation/simulation-2026-10-22.json`, registram:

| Verificação | Resultado |
| --- | --- |
| Modelos e utilitários | 19 testes aprovados naquela revisão |
| Partida, câmera raw/JPEG, avanço, giro e parada | Aprovados |
| Navegação semântica estática, incluindo três poses iniciais | 11 casos aprovados |
| Móvel reposicionado antes e durante a missão; perda das CCTVs | 3 casos aprovados |
| Maior erro de posição nos casos positivos estáticos | 0,2864 m |
| Inferência nas CCTVs da Orin, 673 imagens 1280×720 | Mediana de 192 ms por imagem |

Os casos negativos incluem objeto ausente, hipótese externa falsa,
destino ocupado, cancelamento e expiração de memória. A avaliação usa a
posição verdadeira do Gazebo; essa posição não alimenta o detector visual.
Esses são resultados históricos identificados, não uma nova execução da
bateria GPU a cada alteração de documentação ou teste público.

## Conferência do pacote em 23/10/2026

A verificação para publicação, preservada no acervo local
`extras/validation/simulation-release-2026-10-23.json`, foi realizada em uma cópia com apenas os arquivos publicáveis, sem `extras/`.
Os oito pacotes compilaram e os testes públicos tiveram **177 aprovações e
uma verificação ignorada**, referente ao conversor privado de assets.
A simulação por CPU passou por sensores, TFs, avanço, giro e parada.

Na Orin, a aplicação foi reconstruída sobre a base GPU já preparada. O
Compose público ficou saudável e `goto mesa` concluiu a aproximação e a
confirmação visual. Na mesma imagem, passaram novamente **11 casos estáticos
e três de móveis deslocados**, preservando os critérios de aceitação.
O maior erro de posição dos objetos nos positivos estáticos foi 0,1562 m
nesta execução. Os nós instalados e os pesos tiveram seus hashes conferidos.
Isso registra uma execução adicional; não é uma comparação estatística com
o ensaio anterior.

Os testes são executados manualmente com `make test`. A execução acima foi
local, antes da publicação.

## Limites

A aceitação cobre trajetos curtos, mapa parcial, pose inicial conhecida de
AMCL e um arranjo controlado do LabSEA. O detector foi adaptado a imagens
sintéticas. O teste de móvel deslocado não equivale a seguir pessoas ou
objetos em movimento contínuo. O mapa estático pode conservar ocupação antiga.

Massas, atrito, casters e extrínsecas dos sensores ainda têm aproximações.
A simulação é funcional, mas não estabelece fidelidade física de Digital Twin.
A integração básica da Nano tem ensaios próprios; navegação física, giros e
calibração da C920 permanecem pendentes no [inventário](hardware.md).

## Publicação

`make test` executa o build e os testes públicos sem hardware. A bateria
GPU completa exige a Orin e os executores locais de pesquisa. A pasta
`extras/` permanece privada; os resultados necessários para entender esta
entrega foram resumidos neste guia. Os JSONs completos ficam no acervo local
`extras/validation/`.

O checkpoint incluído e os modelos exportados dispensam treino e conversão
de CAD no primeiro uso. Antes de criar uma tag, revise o diff e inclua os
arquivos novos de `src/`, `config/`, `docker/`, `docs/`, `scripts/`,
`foxglove/`, junto com as alterações dos arquivos existentes.
