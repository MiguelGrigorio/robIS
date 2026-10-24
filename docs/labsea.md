# LabSEA e mapa semântico

Referência do mundo 3D: [JoabFelippx/Labsea-3D](https://github.com/JoabFelippx/Labsea-3D).
O projeto de referência reúne a cena Blender, os dados de calibração e os
scripts de reconstrução e exportação. A versão do GLB integrada aqui é
identificada pelo hash em [PROVENANCE.md](../src/robis_gazebo/models/robis_labsea/PROVENANCE.md).

O arquivo `labsea.glb` fornecido ao projeto foi convertido em um ambiente
Gazebo com estrutura fixa e móveis independentes. O original permanece
intacto. A cópia usada para regenerar os assets fica em
`src/robis_gazebo/models/robis_labsea/source/labsea.glb`.

O arquivo descreve uma reconstrução com dimensões estimadas, incluindo
13 cadeiras, 13 bancadas, uma mesa e três armários. A importação preserva
74 grupos de objetos e uma estrutura, com seus nomes e metadados originais.
As unidades são mantidas em metros; os eixos passam de glTF `(x,y,z)` para
Gazebo `(x,-z,y)`, com Z apontando para cima.

## Cenários

Para executar a versão atual com detector visual e CCTVs, siga o
[guia de simulação semântica](semantic-simulation.md). Os mundos e malhas já
estão exportados no repositório. Os comandos de importação, treino e ensaios
históricos descritos neste documento são ferramentas locais em `extras/`.

| Arquivo de configuração | Cenário gerado | Uso |
|---|---|---|
| `config/labsea.yaml` | `labsea.world` | Todos os objetos nas posições fornecidas |
| `config/labsea_semantic.yaml` | `labsea_semantic.world` | Estrutura, duas cadeiras, mesa e armário para ensaio controlado |
| `config/labsea_rearranged.yaml` | `labsea_rearranged.world` | Cadeira reposicionada e uma mesa adicional |

Os caminhos da tabela são relativos a `src/robis_gazebo/`. Os objetos são
estáticos durante cada execução. Reposicioná-los na configuração permite
comparar arranjos; isso ainda não simula móveis sendo empurrados.

Para mover um objeto, edite sua pose em `overrides`:

```yaml
overrides:
  Chair_001: {pose: [2.2, 1.2, 0, 0, 0, 3.1415927]}
```

A ordem é `x, y, z, roll, pitch, yaw`, em metros e radianos. Os pivôs locais
foram preservados: a cadeira gira em torno de sua própria origem. O campo
`include_only` escolhe os móveis incluídos; a estrutura sempre permanece.
`additions` permite incluir modelos existentes, como `model://robis_table`.
As imagens de referência e os objetos adicionais não são fornecidos ao
localizador como posições conhecidas.

Com a imagem básica de simulação construída, gere novamente os mundos:

```bash
make labsea-worlds
```

O comando usa Python/YAML dentro do container. Depois reconstrua a imagem
de experimento ou o workspace de desenvolvimento para instalar os mundos
atualizados. Para importar uma versão nova do GLB, use `assimp-utils` e:

```bash
python3 extras/scripts/import_labsea.py caminho/para/labsea.glb
make labsea-worlds
```

OBJ/MTL e as texturas extraídas são portáteis e não referenciam o computador
de origem. A conversão mantém geometria, UVs e cores difusas; materiais PBR,
transparência e iluminação são aproximações do arquivo Blender original.
As colisões são malhas estáticas, sem calibração física de atrito ou massa
dos móveis. O manifesto registra o hash do GLB e suas estimativas originais.

## O que significa mapa semântico neste projeto

O mapa métrico do SLAM representa espaço livre e obstáculos. A memória
semântica associa instâncias confirmadas a uma classe, posição no frame
`map`, identificador persistente e contagem de confirmações. Ela é atualizada
após concluir e confirmar a aproximação; não é uma anotação automática de
todos os objetos que apareceram na câmera.

O ensaio começa sem memória, consulta `cadeira` e `mesa`, valida as posições
com a referência do cenário e salva o mapa SLAM. Depois reinicia o processo,
carrega o mapa com AMCL e repete as consultas. Exige navegação à memória,
reconfirmação, manutenção dos identificadores e incremento das contagens.
A memória também precisa ficar vinculada ao hash do mapa métrico salvo.

Essa sequência descreve o ensaio histórico embarcado `labsea-check`.
No perfil público atual, as CCTVs preenchem a memória antes da consulta
e o robô reconfirma o objeto ao chegar. A execução da aplicação usa
`make sim-semantic-up` e os pesos já incluídos em `config/models/`;
não exige repetir coleta ou treinamento.

Na Orin, com `robis:jazzy-detector-runtime` já preparada:

```bash
make labsea-check
```

O comando regenera os mundos, compila os oito pacotes na imagem do ensaio e
executa as duas fases. O runtime CUDA é reutilizado. Os scripts do verificador
são montados em leitura apenas; os nós ROS e os modelos vêm da imagem compilada.
O Compose usa o domínio ROS 33 e o driver físico ODrive é recusado pelo teste.
Não requer janela ou tela remota.

O perfil padrão usa **rótulos/máscaras do Gazebo como controle de percepção**
e `mask_projected_cluster` para associar o LiDAR. A profundidade de referência
fica desligada. As posições são calculadas com câmera, LiDAR e TF; a posição
nativa do cenário é usada apenas pelo verificador externo.

Para avaliar o detector de caixas YOLO26 no mesmo fluxo:

```bash
ROBIS_LABSEA_DETECTOR=yolo26 ROBIS_LABSEA_RANGE_ESTIMATOR=bbox_cluster make labsea-check
```

Antes de repetir, arquive `extras/data/labsea-check/` com outro nome. O teste recusa
uma memória prévia para preservar os resultados e manter o início vazio.
As evidências ficam nessa pasta e em `extras/log/labsea-*`; o container é encerrado
ao terminar ou falhar.

O ensaio de cadastro e recuperação após reinício passou no cenário controlado.
Veja resultados, mapa e imagens (acervo local: `extras/validation/labsea-2026-10-14.md`). Essa aprovação
usa o controle de percepção descrito acima, sem comprovar ainda reconhecimento
aprendido nesse cenário.

## Percepção e VLMs

O teste inicial com YOLO26 genérico no cenário terminou sem encontrar a cadeira.
Com rótulos corretos, a associação apenas pela caixa chegou a escolher a
parede atrás da cadeira; a aproximação foi recusada antes de persistir o
objeto. Esses resultados motivaram o controle por máscara, mantendo separadas
a qualidade da percepção e a validade da memória semântica.

SmolVLM-256M e Florence-2-base-ft foram executados na Orin em uma imagem
opcional de pesquisa, sobre seis imagens RGB salvas do ensaio controlado.
Na tarefa de detecção `<OD>`, Florence reconheceu cadeiras, mas chamou de
avião a mesa vista por baixo. Smol respondeu sobre presença: reconheceu
cadeira nas quatro vistas correspondentes e mesa em uma das duas vistas
dela. Essas verificações são exploratórias, sem anotações manuais para
medir recall ou localização. As respostas brutas estão em
`extras/data/visual-detector/florence.json` e `smol.json`.

SmolVLM é candidato a perguntas sobre a cena; Florence-2 oferece tarefas de
detecção com caixas. Descrever um objeto ou responder que ele existe não
comprova sua localização. A integração dessas saídas ao mapa permanece uma
etapa separada, incluindo qualidade, tempo e associação aos sensores.

## Adaptação de um detector visual

O fluxo experimental de adaptação usa YOLO26s com segmentação, com duas
classes: `chair` e `dining table`. A câmera mantém a posição baixa do modelo
do robô. Não há deslocamento da câmera para favorecer o reconhecimento.
O checkpoint resultante já acompanha o repositório. Os comandos abaixo
documentam a pesquisa local; não fazem parte da instalação normal.

Na Orin, os comandos são:

```bash
make labsea-dataset
make labsea-train
make labsea-visual-check
```

O primeiro comando inicia somente a simulação com sensores de anotação,
captura RGB e máscaras semânticas e encerra o container. As posições de
captura são distintas entre treino, validação e teste; vistas próximas
de uma mesma posição permanecem no mesmo conjunto. Paredes e outras vistas
sem objetos também são mantidas. A coleta não dirige um robô físico.
Uma coleta interrompida pode ser retomada com
`ROBIS_DATASET_RESUME=true make labsea-dataset`, após conferir os arquivos
já salvos pelos hashes do manifesto.

O segundo converte as máscaras em polígonos, adapta os pesos pré-treinados,
seleciona o melhor checkpoint pela validação e avalia o conjunto de teste.
Os dados ficam em `extras/data/labsea-training/` e `extras/data/labsea-segmentation/`;
pesos e métricas em `extras/data/visual-detector/`. Esse modelo é especializado
no cenário sintético; os resultados não comprovam generalização a imagens
reais ou a modelos de móveis diferentes.

Para repetir o treino, arquive primeiro `extras/data/labsea-segmentation/` e
`extras/data/visual-detector/labsea-seg-training/`. O comando recusa essas saídas
existentes para preservar o dataset convertido e os checkpoints anteriores.
O container de treino usa 1 GB de memória compartilhada para os carregadores
de imagens e grava as saídas com o usuário do host.

O último comando compila os pacotes e incorpora o checkpoint selecionado
à imagem `robis:jazzy-labsea-visual`, usando `extras/docker/Dockerfile.visual-detector`.
Os pesos entram por um contexto de build separado; não são montados na
execução. A missão aprende e recupera cadeira e mesa usando RGB, máscaras
preditas, LiDAR e TF. O verificador exige ausência do oráculo e do bridge
dos sensores de anotação. A profundidade de referência permanece desligada.

O perfil visual compensa o movimento entre o instante do scan e o da imagem
usando TF e odometria. Aceita até dois pixels por eixo de incerteza na borda
da máscara predita, exige dois retornos próximos em distância e verifica
descontinuidades dos dois lados no scan original. Isso evita tratar a borda
artificial da máscara como evidência de um objeto à frente de uma parede.
As três confirmações temporais continuam obrigatórias.

As evidências dessa missão ficam em `extras/data/labsea-visual-check/` e
`extras/log/labsea-visual-check-*`, preservando o ensaio com oráculo. Para repetir,
arquive primeiro a pasta dessa missão. Não é necessário coletar ou treinar
novamente para repetir o teste com os mesmos pesos.

O cadastro e a recuperação após reinício passaram com esse perfil. Veja
métricas, imagens, VLMs e limitações (acervo local: `extras/validation/labsea-visual-2026-10-14.md`).

## Gravar uma missão em vídeo

Vídeo de uma execução aprovada e relatório (acervo local: `extras/validation/labsea-video-2026-10-15.md`).

Na Orin, com a imagem visual já construída, execute:

```bash
make labsea-video
```

O comando repete o cadastro e a recuperação de cadeira e mesa após reinício,
com os mesmos critérios do teste visual. Cada execução cria uma pasta nova
`extras/data/labsea-video-AAAAMMDD-HHMMSS/`; não substitui os resultados anteriores
nem coleta dados ou treina novamente. Para escolher o nome:

```bash
ROBIS_LABSEA_RUN=minha-gravacao make labsea-video
```

O arquivo `mission.mp4` reúne a câmera real do Gazebo com caixas e máscaras
preditas, o mapa publicado pelo ROS, o laser, o trajeto do robô e as mensagens
da missão. A memória mostra IDs e confirmações lidos do YAML persistido.
Não é uma animação reconstruída a partir dos resultados, nem uma captura
da interface 3D do Gazebo. A vista do mapa cobre de −5 a +5 metros em cada
eixo, adequada a este cenário controlado.

`extras/docker/Dockerfile.video` reutiliza a aplicação compilada e adiciona FFmpeg dentro
do container; não é preciso instalar ROS, CUDA ou codecs no host. O serviço
`extras/compose/video.yml` é um observador: não publica comandos, não envia
metas e não consulta posições nativas ou sensores de anotação. Usa domínio
ROS 36 por padrão, compartilhado somente com a missão desta gravação.

O MP4 usa H.264, 1280×720 e cinco quadros por segundo em tempo de parede,
incluindo inicialização e reinício. O relógio simulado aparece separadamente;
imagem antiga é sinalizada. `video.json` registra mensagens recebidas,
reinícios, eventos, quadros repetidos para manter a duração e hash do vídeo.
`learn-report.json` e `recall-report.json` continuam sendo os verificadores
de sucesso da missão; ter um vídeo não significa que ela passou.

O script encerra os containers ao terminar. Para copiar somente o vídeo
da Orin ao seu computador, substitua a pasta pelo nome mostrado na execução:

```bash
scp usuario@orin:/caminho/do/repositorio/extras/data/minha-gravacao/mission.mp4 .
```

Fontes: [SmolVLM](https://huggingface.co/HuggingFaceTB/SmolVLM-256M-Instruct),
[Florence-2](https://huggingface.co/microsoft/Florence-2-base-ft),
[implementação nativa no Transformers](https://huggingface.co/docs/transformers/v4.57.1/en/model_doc/florence2).
