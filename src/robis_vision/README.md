# robis_vision

Câmera USB, comportamentos visuais e ferramentas de percepção.

No perfil básico, apenas `usb_camera_node` é iniciado. Ele publica
`/camera/image_raw` e `/camera/camera_info` com o mesmo timestamp e referencial.
A captura usa V4L2 com MJPEG por padrão, caminho testado com a C920 na Nano.
O bringup básico configura 640×480 e solicita 30 fps.
`capture_backend` também aceita `gstreamer` e `auto` para outros ambientes.
Captura e preview usam `opencv_threads=1` para limitar a disputa por CPU.

Uma calibração ROS YAML pode ser informada em `camera_info_url`.
Sem calibração, CameraInfo tem `K[0]=0`; não são inventados parâmetros da câmera.
A configuração inicial fica em `robis_bringup/config/usb_cam_basic.yaml`.

Os detectores, benchmarks e seguidor de bola foram preservados.
YOLO/VLM/GPU não fazem parte da primeira execução do robô.

No LabSEA semântico, `yolo26_detector` fornece detecções em
`/semantic_perception/detections` e máscaras para a associação com o LiDAR.
As CCTVs usam o detector dentro de `fixed_camera_stream`, do pacote
`robis_semantic_navigation`. O perfil GPU usa o checkpoint de segmentação
LabSEA das classes `chair` e `dining table`, com
[pesos e manifesto](../../config/models/README.md) incluídos no repositório.
Essa validação é sintética; o desempenho com imagens físicas ainda está pendente.

Quando o Foxglove é iniciado, `image_preview` fornece
`/camera/image_raw/compressed`: JPEG de qualidade 70, limitado a 30 fps de
tempo real. Ele preserva o cabeçalho da imagem original, mantém apenas o
quadro mais recente e comprime somente quando há assinantes. A percepção
continua usando `/camera/image_raw`. Parâmetros: `input_topic`, `output_topic`,
`fps` e `jpeg_quality`. Veja o [guia do Foxglove](../../foxglove/README.md).

A entrada raw do preview usa QoS confiável e fila de um quadro, compatível
com a câmera USB e o bridge atuais. A saída JPEG continua best effort.
Para uma câmera externa que publique best effort, selecione
`input_reliability:=best_effort` no nó de preview.

Veja o [README principal](../../README.md) e [a compatibilidade da Nano](../../docs/jetson.md).
