# robis_semantic_navigation

Recebe consultas por objeto, escolhe uma aproximação pelo Nav2, confirma o
objeto pela câmera e pelo LiDAR e persiste a instância em um mapa semântico YAML.
Esse banco de objetos é separado do mapa de ocupação usado na navegação.

| Componente | Responsabilidade |
| --- | --- |
| `server.py` | Missões, consulta à memória, aproximação, confirmação e cancelamento |
| `object_localizer.py` | Associação das detecções embarcadas com câmera, LiDAR e TF |
| `store.py` | Persistência das instâncias por ambiente e hash do mapa |
| `fixed_cameras.py` | Projeção e associação de observações das câmeras fixas |
| `fixed_camera_stream` | Recepção de vistas RGB, inferência e publicação de observações externas |
| `fixed_camera_replay` | Processamento de gravações para o mesmo contrato de observações |

No perfil LabSEA, as CCTVs preenchem a memória sem uma exploração inicial
do robô. As capturas mantêm seus timestamps; observações atrasadas ou
expiradas deixam de ser usadas. A câmera embarcada e o LiDAR confirmam a
chegada. Se o objeto externo mudar de posição, a missão pode recalcular a
aproximação; perder a posição válida cancela o destino antigo.
Veja [câmeras fixas](../../docs/fixed-cameras.md) e
[objetos móveis](../../docs/moving-objects.md).

Com a simulação semântica iniciada pelo
[guia LabSEA](../../docs/semantic-simulation.md), abra `make sim-semantic-shell`
e envie, dentro do container:

```bash
ros2 run robis_semantic_navigation goto mesa \
  --environment labsea_fixed_fixture --timeout 120
```

A ação ROS é `/goto_object`. Para cancelar, use `make sim-semantic-stop`
no host ou `ros2 run robis_semantic_navigation stop` no terminal ROS.

O pacote é compilado nos perfis `navigation` do robô e `simulation`.
O [perfil físico completo](../../docs/real-navigation.md) usa os mesmos
componentes com relógio real e calibração USB. A imagem básica não inclui
semântica. O detector adaptado passou no LabSEA simulado; imagens reais e
navegação física continuam pendentes.

`make test` executa os testes públicos dos pacotes. Os ensaios extensos
`semantic-check` e `fixed-cameras-mission-check` pertencem ao acervo local
`extras/`. Veja os [resultados públicos](../../docs/initial-release.md),
[o fluxo completo](../../docs/semantic-navigation.md) e o
[README principal](../../README.md).
