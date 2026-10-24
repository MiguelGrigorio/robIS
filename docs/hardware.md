# Hardware e estado da montagem

Atualizado em **22/10/2026**. Plataforma: **NVIDIA Jetson Nano 4 GB
Developer Kit B01**, JetPack 4.6.6, L4T 32.7.6.

Montagem inicial e testes funcionais realizados. O bringup básico ROS 2 foi
testado na Nano; calibração, giros e validação para navegação permanecem
pendentes. Este registro consolida o
inventário informado pelo operador; os relatórios locais do ODrive também
foram consultados. A integração do driver foi posteriormente testada na Nano
por USB, com ROS em IDLE e com um avanço curto supervisionado, descritos abaixo.

## Inventário

| Componente | Modelo | Interface observada | Estado |
|---|---|---|---|
| Computador | Jetson Nano 4 GB B01 | — | Operacional |
| GPS | GY-NEO6MV2, NEO-6M | UART `/dev/ttyTHS1`, J41 | NMEA a 9600 baud; sem fix no laboratório |
| Visor | OLED I²C, provavelmente SSD1306 | `/dev/i2c-1`, `0x3C` | Texto/contador testados; controlador a confirmar |
| IMU | MPU6050 | `/dev/i2c-1`, `0x68` | Identificada, mas I²C instável; não aprovada para navegação |
| LiDAR | YDLIDAR X4 | USB CP210x, `/dev/ttyUSB0` | Identificado, health good, aquisição ROS perto de 9 Hz |
| Câmera | Logitech HD Pro Webcam C920 | USB, `/dev/video0` no teste | ROS em 640×480 perto de 30 Hz; intrínsecos ainda não calibrados |
| Controle de motores | ODrive v3.6 56 V, firmware 0.5.1 | USB CDC, `/dev/ttyACM0` | Sintonia inicial e perfil verificado em FLASH |
| Tração | Dois BLDC de hoverboard, Hall | Dois eixos do ODrive | Ensaios individuais e conjuntos; ida/volta supervisionadas aprovadas |
| Alimentação de tração | Bateria externa | ODrive | Barramento observado em aproximadamente 41,2 V |

Os nomes `/dev/ttyUSB0`, `/dev/ttyACM0` e `/dev/video0` identificam os testes;
não são identificadores persistentes de dispositivo.

## Conexões registradas

Na B01 foi informada alimentação DC de 5 V, com jumper J48 selecionando a
entrada DC. No J50, foram registrados botão Power nos pinos 11/12 e jumper
de desativação de inicialização automática nos pinos 5/6. Antes de reconectar,
conferir as inscrições físicas e a identificação da placa. Não aplicar tensão
externa aos contatos do botão.

| Pino físico J41 | Sinal | Destino registrado |
|---|---|---|
| 1 | 3,3 V | IMU/OLED, conforme especificação dos módulos |
| 2 | 5 V | VCC do GPS GY-NEO6MV2, se o módulo aceitar 5 V |
| 3 | I²C SDA | MPU6050 e OLED em paralelo |
| 5 | I²C SCL | MPU6050 e OLED em paralelo |
| 6, ou outro GND como 9/14 | Terra | GND dos módulos |
| 8 | UART TX | RX do GPS, opcional |
| 10 | UART RX | TX do GPS |

Os fios do GPS foram descritos como vermelho, azul, amarelo e marrom, sem
mapeamento seguro por cor. Seguir as inscrições VCC/GND/TX/RX do módulo.

## Resultados dos sensores

### GPS

Recebidas em `/dev/ttyTHS1`, a 9600 baud, as sentenças `$GPRMC`, `$GPGGA`,
`$GPGSA`, `$GPGSV` e `$GPGLL`. Nos testes internos: RMC `V`, GGA fix `0` e
zero satélites usados. Falta obter fix em área aberta e validar posição,
precisão e disponibilidade.

### OLED e MPU6050

O OLED em `0x3C` exibiu texto por escrita direta via `smbus`. A biblioteca
Adafruit instalada apresentou `NameError: I2C is not defined` no Python 3.6.
Faltam confirmar o controlador e integrar a interface do robô.

A MPU6050 respondeu `WHO_AM_I=0x68`, mas apresentou NACK e timeout Tegra-I²C.
No ensaio nominal de 60 s (69,44 s reais), houve 678 leituras válidas em
818 tentativas, 140 erros (17,11%) e intervalo máximo de 10,29 s entre
leituras. Mover os cabos alterou a taxa de sucesso; a causa não foi comprovada.
Verificar fiação, soldas, alimentação, pull-ups e estabilidade do barramento.
**Não incluir essa IMU na fusão de navegação enquanto estiver instável.**

### YDLIDAR X4

SDK YDLIDAR v1.1.21 identificou X4, firmware 1.10 e hardware 1. Comunicação
a 128000 baud, `SingleChannel=false`, sample rate reportada de 5,00 k,
aproximadamente 552–558 pontos por varredura e health good. O campo
`scanFreq [0.00]` não foi confirmado como frequência efetiva.

O fluxo `/scan` já foi recebido pelo ROS 2; faltam comparação das distâncias
com referências externas e medição prolongada da frequência. O arquivo
[`ydlidar_real.yaml`](../src/robis_bringup/config/ydlidar_real.yaml) já usa
128000 baud, sample rate 5 e `isSingleChannel=false`. A aquisição ROS mediu
aproximadamente 9 Hz; `frequency=10.0` é uma solicitação, não uma frequência
garantida. O fator herdado `scan_gain=0.88` foi retirado do perfil: o padrão
agora é 1,0, preservando as distâncias do SDK até existir medição externa.
Inversões, extrínsecas e limites de alcance ainda precisam de validação física.

### Logitech C920

Captura em `/dev/video0` confirmada pelo operador. Modos anunciados por V4L2:

| Resolução | YUYV | H.264 | MJPEG |
|---|---:|---:|---:|
| 640 × 480 | 30 FPS | 30 FPS | 30 FPS |
| 1280 × 720 | 10 FPS | 30 FPS | 30 FPS |
| 1920 × 1080 | 5 FPS | 30 FPS | 30 FPS |

São modos anunciados, não medições sob carga. A captura conjunta foi
ensaiada em 640 × 480 e 1280 × 720; os resultados estão na seção de imagem
compilada abaixo. Latência física, exposição e estabilidade prolongada ainda
precisam de avaliação. O perfil ROS permanece em 640 × 480, solicitando 30 FPS e
selecionando MJPEG via V4L2. A compressão JPEG para o Foxglove ocorre
depois da captura e não configura o formato do transporte USB.

Faltam calibração intrínseca/distorção (por exemplo, ChArUco), extrínsecas
em relação ao `base_link` e avaliação de exposição/foco. Os NPZ em
`config/calib/` pertencem às câmeras do teto e não calibram a C920.

## ODrive e locomoção

Placa v3.6 56 V, firmware **0.5.1**, USB `1209:0d32`, serial
**`207C399D4D4D`**. Os seguintes parâmetros foram salvos e verificados após
reboot no projeto de calibração:

| Parâmetro | Ambos os eixos |
|---|---:|
| `controller.config.control_mode` | 2, velocidade |
| `controller.config.input_mode` | 2, rampa |
| `controller.config.vel_gain` | 10 |
| `controller.config.vel_integrator_gain` | 0,75 |
| `controller.config.vel_limit` | 0,931820 volta/s, aproximadamente 0,5 m/s |
| `controller.config.vel_ramp_rate` | 0,75 volta/s², aproximadamente 0,402 m/s² |
| `controller.config.inertia` | 0 |
| `motor.config.current_lim` | 2,5 A |
| `motor.config.current_lim_margin` | 1 A |
| `encoder.config.bandwidth` | 50 |
| `controller.config.enable_vel_limit` | 1 |
| `controller.config.enable_overspeed_error` | 1 |
| `controller.config.vel_limit_tolerance` | 1,2 |
| `config.watchdog_timeout` | 2 s |
| `config.enable_watchdog` ao ligar/em IDLE | 0; habilitado e alimentado pelo software ativo de calibração |
| Seis `config.startup_*` | 0, sem movimento automático na partida |

Parâmetros elétricos preservados:

| Parâmetro | Eixo 0 | Eixo 1 |
|---|---:|---:|
| `motor.config.phase_resistance` | 0,512411 Ω | 0,507638 Ω |
| `motor.config.phase_inductance` | 0,000658 H | 0,000639 H |
| `motor.config.pole_pairs` | 15 | 15 |
| `encoder.config.cpr` | 90 | 90 |
| `encoder.config.mode` | 1, Hall | 1, Hall |
| `motor.config.torque_constant` | 2,0675 | 2,0675 |

A constante de torque não foi medida externamente. O termistor dos motores
está desabilitado e não houve caracterização térmica prolongada. Frenagem e
regeneração continuam exigindo validação. `current_lim` é limite de corrente
comandada; não demonstra um corte físico instantâneo em 2,5 A.

Geometria de projeto: raio de roda de 0,0854 m e distância entre rodas de
0,36 m, ainda sem verificação metrológica. Avanço observado: eixo 0 negativo
e eixo 1 positivo; recuo com sinais inversos. Falta confirmar a associação
esquerda/direita e validar comandos angulares.

### Percurso supervisionado aprovado

Referências de 0,5 m/s na ida e 0,2 m/s na volta. O operador observou
percursos retos nos dois sentidos.

| Medida por encoder | Eixo 0 | Eixo 1 |
|---|---:|---:|
| Distância de ida | 1,17445 m | 1,17455 m |
| Velocidade de ida pela posição | 0,46868 m/s | 0,46678 m/s |
| Velocidade de retorno pela posição | 0,17252 m/s | 0,17181 m/s |
| Erro absoluto final de retorno | 1,79 cm | 2,39 cm |

Duração de 12,52 s, 206 amostras, corrente absoluta máxima de 1,712 A e
diferença máxima de percurso entre rodas de 1,18 cm. Esses dados derivados
de encoder não estabelecem posição absoluta, orientação ou ausência de
escorregamento.

### Código e evidências

Projeto de calibração local: `/home/miguelgads/Projects/calib-robis`.
Na Jetson: `/home/jetson/robis-odrive`. Referências dentro desse projeto:

- `docs/permanent-profile.md`: descrição do perfil e do ensaio.
- `profiles/ground_05mps.json`: perfil exportado e política do software ativo.
- `reports/commissioning-05mps-validated/validation.json`: ensaio aprovado.
- `roundtrip.csv`, `roundtrip.png`, `saved-profile.json` e `final-state.json`
  no mesmo diretório de relatórios.

O ensaio registra `configuration_saved=false`, pois não grava FLASH durante
o percurso. A confirmação de gravação e verificação após reboot está no
relatório separado `saved-profile.json`. Foram informados 42 testes
automatizados aprovados na Jetson com Python 3.6.9; eles pertencem ao projeto
de calibração e não são resultados de navegação ROS 2.

Diagnóstico somente leitura na Jetson, sem executar o percurso:

```bash
cd ~/robis-odrive
python3 -m robis_odrive.commissioning
python3 -m robis_odrive --output reports/check-after-power-cycle.json
```

## Integração pendente no is-robis

O perfil persistido na placa não inclui toda a política executada pelo
software da Jetson. A sincronização de percurso, guardas de corrente,
temperatura e diferença de rodas e política de parada do projeto de
calibração não são automaticamente herdadas pelo driver ROS. O driver atual
já implementa as verificações de perfil, watchdog e guardas descritas abaixo;
as pendências restantes são principalmente de validação física.

| Item | Situação no código ROS atual | Próxima validação/alteração |
|---|---|---|
| Partida | Conecta sem calibrar ou engatar | Preservar partida sem movimento; comando não nulo ainda pode engatar |
| Watchdog da placa | Habilitado/alimentado em sessão; desligado após IDLE confirmado | Expiração real por falha do processo ainda não ensaiada |
| Timeout de comando | Para após 0,5 s; libera por janela de parada ou prazo de 2 s | Comparar frenagem e distância com referência externa |
| Odometrias | Perfil físico em `encoder`, sem fator empírico 100 | Validar distância real e giros |
| Polaridade | Avanço ROS testado; giros bloqueados por `allow_angular_motion=false` | Confirmar esquerda/direita e sentido angular |
| Perfil e identidade | Serial, firmware, estados e perfil verificados antes de conectar/engatar | Repetir a inspeção após alterações de hardware ou comissionamento |
| Monitoramento ativo | Guardas de corrente, tensão, temperatura, estado e percurso; correção apenas em reta | Validar fisicamente a guarda adaptada a curvas |
| Sensores adicionais | GPS, OLED e MPU6050 não são iniciados no perfil básico | Integrar separadamente; manter IMU instável fora da estimativa |
| Câmera e LiDAR | Bringup conjunto passou na Nano, com imagem e scan recebidos | Calibrar, medir TFs e validar distâncias e operação prolongada |

A sincronização de distâncias para ida/volta reta não pode ser aplicada
sem adaptação a curvas: os percursos esperados das rodas são diferentes.
O ensaio do projeto de calibração não aprova o launch ROS nem a navegação.

Prioridades: confirmar esquerda/direita, curvas e distâncias reais; calibrar C920 e TFs
de câmera/LiDAR; estabilizar a IMU; obter fix externo do GPS. Depois,
validar estimativa de estado e navegação com supervisão e critérios de parada.

## Integração ROS na Nano — 22/10/2026

A inspeção USB confirmou perfil, calibração e serial. Além de `0.5.1`, a API
informa `fw_version_unreleased=1`; o perfil esperado registra explicitamente
esse identificador. Nenhum parâmetro foi salvo em FLASH nesta integração.

O driver permaneceu por 10,12 s em IDLE, publicando odometria a 20,01 Hz,
sem publicar comandos ou solicitar engate. O teste seguinte, com referência
de 0,05 m/s e pernas de 0,8 s, não passou no critério de deslocamento.
Não houve confirmação visual desse primeiro percurso. A falha foi preservada;
não foi classificada como validação de ida/volta pelo ROS.

Depois, com autorização para apenas avançar, foi executada referência de
0,1 m/s por 1,5 s e parada. O deslocamento final médio indicado pelos encoders
foi aproximadamente 8,7 cm; os percursos individuais foram cerca de 8,48 e
8,93 cm. O maior Iq observado foi 0,93 A. Ambos os eixos terminaram em IDLE,
watchdog desligado e perfil correspondente ao esperado. Essa aprovação é
do ensaio limitado de avanço/parada, sem medição externa de trajetória.
A referência nominal corresponderia a 15 cm sem rampas/atrasos; a precisão
na faixa de baixa velocidade ainda precisa de avaliação.

O ensaio usou `odom_calc_hz=20`, timeout de comando de 0,25 s e liberação
ociosa de 1,5 s. O perfil de operação também solicita 20 Hz de supervisão e
odometria no host; isso não altera o laço interno do ODrive. Seus padrões de
timeout e liberação continuam em 0,5 s e 2 s. A taxa efetiva sob carga ativa
prolongada ainda precisa ser medida.

Os executores e evidências ficam em `extras/scripts/check_odrive_idle.py`,
`extras/scripts/check_odrive_motion.py` e
`extras/validation/odrive-integration-2026-10-22/`. O código testado foi
carregado em um diretório de ensaio na Nano e executado com a imagem Jazzy
existente, sem substituir a instalação de calibração ou instalar autostart.
Veja [controle, proteção e parâmetros](odrive.md).

## Imagem compilada e sensores em conjunto — 22/10/2026

A imagem **`robis:jazzy-basic-20261022`**, também marcada como
`robis:jazzy-basic`, foi construída nativamente na Nano com
`docker/Dockerfile.robot --target robot`. Tem aproximadamente 1,95 GB,
seis pacotes instalados e SDK do YDLidar compilado. A execução usou o código
da própria imagem, sem pasta `src/` ou compilação na partida. A biblioteca
Python ODrive é 0.6.11.post0; o firmware continua 0.5.1.

O bringup completo rodou por 15 s, com câmera, LiDAR, TFs, Foxglove e
ODrive conectada em IDLE:

| Fluxo recebido | Frequência na janela completa |
|---|---:|
| Imagem BGR 640×480 | 29,86 Hz |
| CameraInfo | 29,99 Hz |
| Preview JPEG | 28,06 Hz |
| LaserScan e PointCloud | 9,00 Hz |
| Odometrias | 20,00 Hz |

Os nove frames fixos do modelo foram recebidos, imagem e CameraInfo tiveram
timestamps correspondentes e a odometria permaneceu em zero. Não foram
observados comandos de movimento. Ao terminar, ambos os eixos estavam em
IDLE, watchdogs desligados e perfil correspondente, sem gravar FLASH.
Os estados móveis das rodas/casters continuam como pendência separada.

A C920 usa V4L2/MJPEG, solicitando 30 FPS, e captura/preview limitam OpenCV
a uma thread por processo. O ensaio em 1280×720 entregou 27,07 CameraInfo/s,
14,66 imagens BGR/s ao observador e 11,33 JPEG/s nessa carga. O padrão fica
em 640×480; os modos anunciados pelo dispositivo não garantem essas taxas
na aplicação inteira. CameraInfo permanece não calibrado, com `K[0]=0`.

A inconsistência de escala entre LaserScan e PointCloud foi corrigida.
O perfil usa ganho 1,0; um teste separado com ganho 0,88 e bins vazios
infinitos confirmou que ambos os tópicos recebem a mesma correção.
Isso verifica consistência de software, não a exatidão física do LiDAR.

Também foram testados em software os dois vínculos possíveis de eixos,
preservando os sinais físicos de avanço, e o yaw positivo na convenção ROS.
O vínculo físico segue desconhecido, portanto giros continuam bloqueados.
Falhas nos serviços de reset/calibração agora também encerram a sessão.

Os 86 testes offline e 15 testes ROS passaram. Evidências, falhas anteriores
e identificação exata da imagem ficam em
`extras/validation/sensors-2026-10-22/`; logs em
`extras/log/sensors-2026-10-22/`. A imagem e `~/robis/run_robot.sh` ficaram
disponíveis na Nano; os containers de teste foram encerrados. Esse ensaio
curto confirma o fluxo básico, com calibração, giros, distância externa,
dinâmica de navegação e operação prolongada ainda pendentes.
