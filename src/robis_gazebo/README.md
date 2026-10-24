# robis_gazebo

Contém os mundos e a comunicação entre Gazebo e ROS.
Entra no perfil `simulation`; as imagens do robô físico não incluem Gazebo.

`simulation.launch.py` carrega o robô e os sensores. Os tópicos incluem
`/scan`, `/odom`, `/camera/image_raw` e `/camera/camera_info`.
O Gazebo recebe movimento em `/cmd_vel`.

Comece com `make sim-up` na raiz do repositório.
Use `make sim-check` para conferir sensores, TF e movimento; `make sim-gui`
abre a janela. Os sensores extras de pesquisa têm bridges separados e só
iniciam com `enable_ground_truth_sensors:=true`.
Veja o [guia de simulação](../../docs/simulation.md) e os
[resultados públicos](../../docs/initial-release.md).

O LabSEA foi atualizado a partir do GLB fornecido em outubro/2026: 74 grupos
de objetos, além da estrutura. A referência do mundo 3D é
[JoabFelippx/Labsea-3D](https://github.com/JoabFelippx/Labsea-3D); a versão
integrada está identificada em [PROVENANCE.md](models/robis_labsea/PROVENANCE.md).
O cenário completo preserva suas posições;
os layouts controlados continuam aplicando as poses de seus YAMLs. Os
ensaios de outubro/2026 já usaram os modelos atualizados no cenário controlado;
veja [o escopo da versão inicial](../../docs/initial-release.md).
Fontes Blender e a revisão anterior ficam em `extras/assets/`.

Os módulos `robis_gazebo.runtime` instalam as sessões usadas pela simulação
manual. Os executores de ensaios que reutilizam essas sessões ficam em
`extras/scripts/`; a imagem de operação não depende desses executores.

O perfil público de LabSEA com câmeras fixas, YOLO, memória e Nav2 usa
`make sim-semantic-up`. Ele requer a base GPU e o checkpoint descritos no
[guia de uso](../../docs/semantic-simulation.md). O mundo completo e o arranjo
controlado são distintos; a aceitação atual cobre o arranjo controlado.
