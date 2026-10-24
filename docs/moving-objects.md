# Objetos móveis pelas CCTVs

As CCTVs calibradas fornecem as posições dos objetos ao mapa semântico.
O mapa métrico usado pelo Nav2 representa áreas livres e obstáculos; ele
não é o banco de posições dos objetos. Durante uma missão, câmera e LiDAR
do robô continuam necessários para confirmar o objeto e a distância final.

O primeiro cenário é uma mesa do LabSEA sendo deslocada lentamente e
depois parada. Não é ainda acompanhamento de pessoas ou de um alvo que
se move indefinidamente.

Os três casos e a repetição dos onze casos estáticos estão documentados no
relatório com imagens e resultados (acervo local: `extras/validation/moving-objects-2026-10-18.md`).
Os casos foram repetidos na conferência de outubro/2026, resumida na
[entrega com simulação](initial-release.md).

Para a mesa, o ponto projetado vem da região mais espessa da máscara
de segmentação (`anchor_method: mask_incenter`). Isso evita que o centro
da caixa oscile quando o detector inclui ou exclui pernas. Sem uma máscara
com região suficiente, a observação é descartada, sem substituir esse
ponto pelo centro da caixa. A classe `chair` conserva o centro da caixa.
Stream ao vivo e replay usam a mesma função para escolher o ponto.

## Comportamento

O comando continua igual, dentro de `make sim-semantic-shell`:

```bash
ros2 run robis_semantic_navigation goto mesa \
  --environment labsea_fixed_fixture --timeout 90
```

Quando a origem é uma CCTV, a missão conserva o ID selecionado, consulta
sua posição durante a aproximação e recalcula a rota se o deslocamento
acumulado desde o último planejamento atingir 0,35 m. Primeiro cancela
a ação Nav2 anterior e espera seu resultado terminal. Só então escolhe
uma nova aproximação com caminho válido e distância de segurança.

Pequenas alterações não provocam uma meta a cada frame. O limite é de
dez replanejamentos, além do timeout total da missão. Se a posição do ID
expirar ou desaparecer, o destino antigo é cancelado e a missão termina
com `NOT_FOUND`. Outra mesa ou cadeira não substitui automaticamente o
ID perdido. Uma falha de caminho retorna `NAVIGATION_FAILED`.

A confirmação embarcada usa a posição atual como referência e atualiza
o mesmo registro, sem fazer média com sua posição antiga. A quarentena
de uma hipótese efetivamente recusada pelo robô continua existindo.

Essas regras se aplicam a registros externos válidos. A busca comum por
um objeto sem memória e os registros estáticos aprendidos pelo robô
mantêm o comportamento anterior.

Os parâmetros do servidor estão em
[`semantic_navigation.yaml`](../src/robis_semantic_navigation/config/semantic_navigation.yaml):

| Parâmetro | Padrão | Uso |
|---|---:|---|
| `live_external_targets` | `true` | Atualizar uma missão iniciada por memória externa |
| `target_replan_distance_m` | `0.35` | Deslocamento acumulado para trocar a aproximação |
| `target_max_replans` | `10` | Limitar tentativas quando o objeto continua mudando |

## Reproduzir o ensaio

Os verificadores desta seção pertencem às ferramentas locais `extras/`.
O comportamento de atualização dos destinos faz parte da aplicação pública;
veja os [resultados registrados](initial-release.md).

Na Orin preparada conforme o [guia principal](semantic-simulation.md),
reconstrua a aplicação e rode:

```bash
make sim-semantic-build
make sim-semantic-moving-check
```

O teste usa a imagem compilada, quatro câmeras RGB com as calibrações
originais, YOLO26 com os pesos já existentes e Nav2 real no Gazebo.
Não monta o código da aplicação nem usa dispositivos físicos. O script
de ensaio é montado somente para leitura, como na aceitação estática.
Usa domínio ROS 70 e partição Gazebo próprios.

Os três casos são: deslocar a mesa durante a navegação, deslocá-la antes
do comando e perder o processamento das CCTVs durante a aproximação.
O deslocamento positivo é de 0,9 m, dentro da cobertura observada do detector.
O verificador move o modelo visual e de colisão pelo serviço do Gazebo.
Essas coordenadas nunca são publicadas como detecções ou destinos: a
atualização da memória precisa vir da inferência nas imagens.

Resultados, imagens, posições e estados das ações ficam em
`extras/data/semantic-moving/`. Uma execução não sobrescreve evidências anteriores:

```bash
make sim-semantic-moving-check SEMANTIC_MOVING_RUN=semantic-moving-02
```

Para repetir os onze casos estáticos na nova imagem:

```bash
make sim-semantic-check SEMANTIC_ACCEPTANCE_RUN=semantic-static-after-moving
```

## Limites deste primeiro cenário

A associação das CCTVs é geométrica por classe e proximidade. Ela não
faz reidentificação por aparência: cruzamentos entre objetos semelhantes,
saltos maiores que a janela de associação e oclusões longas podem perder
ou trocar IDs. O ensaio de movimento usa uma única mesa e pequenos
incrementos, abaixo dessa janela. Perder o ID interrompe a aproximação.
Dentro da janela de distância, um rastreamento já confirmado tem prioridade
sobre uma hipótese nova de uma só vista. Isso evita que uma caixa duplicada
do detector tome as observações do ID estabelecido. Essa prioridade não
dispensa o suporte atual de múltiplas câmeras para publicar uma posição.
O perfil exige suporte de pelo menos duas câmeras na janela de fusão.
Se apenas uma continuar detectando o objeto, o registro deixa de ser
publicado e expira. O prazo não é renovado pela mera existência de uma
posição antiga. Houve uma rodada com 1,2 m que perdeu o registro válido
antes da correção da associação de IDs e terminou com cancelamento.
O ensaio positivo de 0,9 m não estabelece um deslocamento máximo do algoritmo.

A localização continua sendo uma estimativa por raio e plano de altura
da classe; não há profundidade medida nas CCTVs. O alinhamento físico
das calibrações ao mapa ainda precisa ser validado fora da simulação.
A região espessa é uma heurística para este tipo de móvel; não identifica
formalmente o tampo e não garante precisão para mesas de outra geometria.

O mapa parcial de SLAM é salvo antes do deslocamento e pode conservar
a ocupação antiga do móvel. As costmaps recebem obstáculos atuais pelo
LiDAR, mas esse teste não remove ocupação antiga do mapa estático nem
valida remapeamento contínuo. Isso pode impedir caminhos que ficaram livres.

Mover o modelo por incrementos não simula forças de uma pessoa empurrando
o móvel. O ensaio verifica percepção, memória, cancelamento, replanejamento
e reconfirmação, sem validar dinâmica de contato ou segurança física.
