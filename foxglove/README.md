# Foxglove para o robIS

O arquivo `robis.json` é um layout pronto para importar. Ele usa os tópicos
comuns ao robô e à simulação. A observação funciona por padrão; o controle
manual precisa ser habilitado no bridge.

## Conectar à simulação

Na raiz do workspace:

```bash
make sim-up
```

Aguarde o build e o spawn do robô. O Compose inicia o pacote oficial
`foxglove_bridge` junto da simulação, ouvindo em `127.0.0.1:8765`.
Use o Foxglove atual no desktop ou em [app.foxglove.dev](https://app.foxglove.dev/).
Selecione **Open connection → Foxglove WebSocket** e informe:

```text
ws://localhost:8765
```

No menu **Layouts → Import from file**, selecione `foxglove/robis.json`.
Depois de atualizar esse arquivo, importe novamente para aplicar as alterações
ao layout salvo no cliente.
O layout contém cinco abas:

- **Visão geral:** URDF e lidar em 3D, câmera, velocidade linear e angular
  comandadas comparadas com a odometria.
- **Odometria:** mensagem completa e evolução de X/Y.
- **Sensores:** estados das juntas, CameraInfo e relógio da simulação.
- **Navegação:** mapa, lidar, caminho planejado, destino do teste e pose AMCL.
  Requer `make nav-check` ou `make sim-nav` com um mapa válido; os costmaps
  ficam disponíveis, desligados inicialmente para manter o mapa legível.
- **Controle manual:** robô/LiDAR, câmera e painel Teleop.

O referencial 3D inicial é `odom`; o layout funciona antes de existir um mapa.
As malhas STL deste robô usam Z como eixo vertical; ambos os painéis 3D
definem `scene.meshUpAxis=z_up`. Se apenas o chassi aparecer de lado, confira
**Scene → Mesh "up" axis → Z-up** nas
[configurações do painel 3D](https://docs.foxglove.dev/docs/visualization/panels/3d#scene).
Os eixos e nomes dos referenciais ficam ocultos inicialmente para não cobrir
o robô. Para inspecioná-los, use as configurações de **Transforms** no painel 3D.
Os gráficos de velocidade usam limites fixos adequados aos parâmetros atuais
de navegação, evitando ampliar o ruído numérico da odometria em repouso.
Um aviso na série de comando pode indicar que ainda não chegaram mensagens
de `/cmd_vel`; confira a descrição do aviso.

## Imagem e tráfego de rede

O launch do bridge também inicia `robis_vision image_preview`. Os painéis
de imagem usam `/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`),
com JPEG de qualidade 70 e limite de **30 fps de tempo real**. A taxa efetiva
depende da câmera, da simulação e da capacidade de processamento. Para reduzir:

```bash
ros2 launch robis_bringup foxglove.launch.py preview_fps:=15.0
```

Esse comando inicia o bridge; encerre uma instância anterior que use a mesma
porta antes de executá-lo. Reconstrua a imagem depois de atualizar os pacotes.
O stream original `/camera/image_raw` mantém a resolução e frequência usadas
pelo detector. A prévia preserva seu timestamp e referencial, guarda somente o
quadro mais recente e só comprime quando há assinantes. Não acumula quadros
atrasados nem inventa imagens para alcançar 30 fps.
Se iniciar um segundo bridge no mesmo domínio ROS, use outro `node_name` e
`enable_image_preview:=false` para reutilizar a prévia já existente.

Uma imagem RGB de 640×480 ocupa 921.600 bytes antes da compressão; a 30 fps,
são cerca de 27,6 MB/s apenas para a câmera. Enviar o tópico raw pelo Foxglove
pode esgotar a fila do servidor em uma conexão lenta. Ao atualizar, reimporte
o layout e confira se os painéis selecionaram o tópico `compressed`. Aumentar
`message_backlog_size` sozinho não resolve um fluxo maior que a rede consegue
transportar. O bridge exclui `/camera/image_raw` de sua lista de tópicos para
evitar que um layout antigo volte a transmitir a câmera sem compressão.
O tópico permanece disponível aos nós ROS; outros tópicos de imagem bruta
continuam disponíveis para diagnóstico.
Por padrão, o bridge habilita observação do grafo e leitura de malhas.
Para desativar o bridge na simulação:

```bash
ROBIS_FOXGLOVE_ENABLED=false make sim-up
```

Isso recria o serviço. Também é possível manter essa escolha no `.env`.
O layout pode ser usado com gravações; nesse caso não é necessário um bridge.

## Controle manual

O [painel Teleop](https://docs.foxglove.dev/docs/visualization/panels/teleop)
publica `geometry_msgs/msg/Twist` em `/cmd_vel_teleop`, a 20 Hz. O encaminhador
`foxglove_teleop.py` entrega comandos limitados em `/cmd_vel`.
Para habilitar no launch de um bridge separado:

```bash
ros2 launch robis_bringup foxglove.launch.py use_sim_time:=true \
  port:=8766 node_name:=foxglove_manual enable_teleop:=true
```

Reconstrua a imagem antes de usar os novos pacotes. Se já houver uma prévia
no mesmo domínio, acrescente `enable_image_preview:=false`. O exemplo usa
loopback; para acesso pela LAN/Tailscale, acrescente `address:=0.0.0.0`.

Reimporte o layout, abra **Controle manual** e pressione **Stop** uma vez
antes de usar as setas. Avanço/ré: ±0,15 m/s; giro: ±0,4 rad/s.
**Stop on release** está ativo. Se os comandos cessarem durante o movimento,
o encaminhador publica parada após 0,5 s, com verificação a cada 0,05 s de
tempo real, e exige outro Stop antes de aceitar movimento novamente.
Ele limita a velocidade a ±0,2 m/s e ±0,6 rad/s e descarta eixos não planares.
Esses limites se aplicam ao caminho `/cmd_vel_teleop → /cmd_vel`.

Antes de dirigir manualmente, cancele uma missão ativa com
`ros2 run robis_semantic_navigation stop`. O encaminhador não é um árbitro
entre operadores e Nav2: não envie os dois tipos de comando simultaneamente.
O Stop do painel zera a velocidade manual; não cancela ações Nav2.

`enable_teleop` fica desligado por padrão, inclusive no launch usado pelo
robô físico. Quando habilitado, o bridge anuncia `clientPublish`; serviços
e alterações de parâmetros continuam desligados. No bridge 3.6.0 ensaiado,
`client_topic_whitelist` não restringiu a publicação no teste isolado, por
isso esse parâmetro não é usado como garantia de filtragem. Use o tópico
configurado no layout para passar pelo encaminhador e seus limites.

Os testes estão registrados em
teleop no Foxglove (acervo local: `extras/validation/foxglove-teleop-2026-10-18.md`).

## Conectar ao robô

A imagem pronta também inclui o bridge; ele fica desligado por padrão.
Com o container do robô já em execução, abra um terminal separado:

```bash
docker exec -it robis /usr/local/bin/robis-entrypoint \
  ros2 launch robis_bringup foxglove.launch.py
```

No computador, encaminhe a porta pelo SSH:

```bash
ssh -N -L 8765:127.0.0.1:8765 usuario@jetson
```

Conecte ao mesmo `ws://localhost:8765` e importe o mesmo layout.
Se também estiver observando uma simulação local, use uma porta local diferente
no túnel, por exemplo `-L 8766:127.0.0.1:8765`, e conecte a `ws://localhost:8766`.
O bridge usa o domínio ROS do container, sem misturar os domínios 27 e 28.
Esse comando inicia o bridge e a prévia de imagem; o robô já iniciado continua
com seu próprio launch. Ctrl+C no terminal do bridge encerra a observação.

No perfil físico atual, `/clock` não existe e a publicação de `/joint_states`
pela ODrive ainda não foi implementada. Esses painéis ficarão sem mensagens.
Sem calibração física fornecida, CameraInfo informa `K[0]=0`;
o painel de imagem usa a imagem original, sem retificação.

## Como está implementado

ROS 2 usa descoberta pelo middleware e não possui o master do ROS 1.
O bridge participa do mesmo domínio ROS e fornece os dados ao cliente
Foxglove. Não existe um servidor de socket escrito para este projeto.
O launch e a configuração estão em `src/robis_bringup/`.

As configurações dos painéis foram geradas a partir dos modelos oficiais de
layout do Foxglove SDK 0.59.0 e organizadas no formato JSON de exportação.
O repositório não precisa instalar o SDK para usar ou editar o layout.

O teste de conexão usou o bridge Jazzy **3.6.0**: recebeu mensagens de nove
tópicos e recuperou a malha do chassi. Essa versão negocia o protocolo
`foxglove.sdk.v1`; use um cliente Foxglove atual. Em 18/10/2026, uma captura
do cliente confirmou a exibição do URDF, câmera e gráficos de odometria no
LabSEA. A captura revelou nomes de referenciais sobrepostos; o layout foi
ajustado para ocultá-los. Essa revisão ainda precisa ser reimportada e
conferida visualmente no cliente.
Durante a missão Nav2, o protocolo também entregou mapa, caminho planejado,
pose AMCL, destino e os dois costmaps. Log: `log/foxglove-navigation-check.json`.

Referências: [conexão ROS 2](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2?modality=live),
[layouts](https://docs.foxglove.dev/docs/visualization/layouts) e
[configuração do bridge](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md).
