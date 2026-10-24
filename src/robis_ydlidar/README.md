# robis_ydlidar

O launch do driver publica os dados; as TFs fixas são responsabilidade do
URDF comum em `robis_description`, carregado pelo bringup.

Driver ROS 2 do lidar físico. Publica LaserScan em `/scan`.

Durante o build, o Docker baixa o SDK de
[YDLIDAR/YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK), fixado no commit
`7c0a2328fd85b65960c37287a67b0edbee8caa4b`, e compila a biblioteca.
A imagem final contém a biblioteca instalada; não é necessário clonar ou
instalar o SDK na Nano. Veja [build e deploy](../../docs/build.md).

Os parâmetros do robô ficam em `robis_bringup/config/ydlidar_real.yaml`.
O perfil básico inicia o driver diretamente e usa o URDF para a TF do lidar,
evitando dois publicadores para o mesmo referencial.
O launch separado `ydlidar_launch.py` aceita `params_file`; seu arquivo
padrão é genérico. Para usar a configuração X4 do robIS, selecione o YAML
do bringup citado acima.

O X4 foi identificado fisicamente com firmware 1.10, hardware 1 e 128000
baud. A aquisição ROS mediu aproximadamente 9 Hz; solicitar `frequency=10.0`
não garante essa taxa. O perfil usa `scan_gain=1.0`, sem correção empírica de
distância. Ganhos configurados são aplicados tanto a `/scan` quanto a
`/point_cloud`, com filtragem de valores inválidos e fora dos limites.
`invalid_range_is_inf=true` representa bins vazios com infinito; `false`
preserva o zero usado anteriormente. Distâncias, sentido angular e TF ainda
precisam de comparação com referências físicas.

Na simulação, o scan é publicado pelo Gazebo, e este driver não é compilado.

Veja o [README principal](../../README.md).
