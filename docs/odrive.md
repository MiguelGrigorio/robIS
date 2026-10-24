# Controle e odometria da ODrive

O driver compartilha a mesma lógica entre a placa real e uma interface
simulada. O Gazebo continua usando DiffDrive, em outro domínio ROS.
Os testes automatizados usam uma interface substituta. A validação na Nano
é registrada separadamente, incluindo inspeção USB, ROS em IDLE e um
avanço curto supervisionado.

O [inventário de 22/10/2026](hardware.md) registra a ODrive v3.6 56 V,
firmware 0.5.1, serial `207C399D4D4D`, o perfil persistido e o percurso
supervisionado no projeto separado `calib-robis`. Esses ensaios ainda não
validam navegação. O driver ROS agora verifica o perfil antes de habilitar os
motores e incorpora watchdog e guardas de telemetria. Os ensaios físicos do
controle ainda precisam de comparação com medidas externas.

## Organização

- `drive_math.py`: geometria, mapeamento de comandos, unidades, encoders e pose.
- `drive_controller.py`: conexão, calibração, engate, timeout e encerramento.
- `odrive_node.py`: tópicos, serviços e parâmetros ROS.
- `odrive_interface.py`: API Python 0.6.11.post0 com firmware 0.5.1.
- `odrive_simulator.py`: substituto determinístico da mesma interface.
- `drive_safety.py`: identidade, perfil e guardas de percurso/parada.
- `params/ground_05mps.json`: perfil esperado, incluído na imagem.
- `profile_inspection.py`: inspeção USB sem comandos de escrita.

O simulador reproduz polaridades, voltas/s e contagens por revolução do
contrato da API. Não reproduz inércia, atrito, limites de torque, eletrônica
ou latência da placa. Essas características pertencem à validação física.

## Inicialização e comandos

`connect_on_startup=false` deixa a interface desconectada. Comandos são
descartados até uma conexão explícita. Calibração ou engate na inicialização
exigem `connect_on_startup=true`; combinações contraditórias são rejeitadas.

`calibrate_on_startup=true` executa calibração após conectar, antes de engatar.
O padrão é `false`. `engage_on_startup=false` deixa os motores liberados ao
iniciar. Um comando não nulo pode engatá-los depois de conectados; um comando
zero não engata motores liberados. `axis_for_right` é repassado à interface.
`use_preroll=true` habilita a busca de índice antes do engate.

No YAML físico, a conexão é `true`; calibração e engate inicial são `false`.
O deploy apenas transfere a imagem; o launch básico conecta sem engatar,
mas comandos não nulos posteriores podem habilitar os motores.

`allow_angular_motion=false` bloqueia giros no perfil físico até confirmar
os lados das rodas. Um comando angular bloqueado solicita parada, sem
renovar o prazo do comando anterior. A associação atual `axis_for_right=0`
continua provisória; o sinal de avanço dos dois eixos foi confirmado.

Os sinais físicos de avanço ficam em `motor_signs` no perfil: eixo 0 negativo,
eixo 1 positivo. Eles são independentes de `axis_for_right`; trocar a escolha
do lado não inverte o avanço. O retorno dos encoders é normalizado com os
mesmos sinais. O padrão `encoder_yaw_sign=1.0` usa a convenção ROS: giro
positivo tem roda direita mais rápida que a esquerda. Isso só representa o
giro físico correto depois de identificar os lados. A seleção permanece
bloqueada para movimento angular enquanto essa identificação estiver pendente.
Referência: [convenções de eixos e rotação da REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst).

Antes de conectar e antes de cada engate, são conferidos serial, hardware,
firmware, configuração carregada, estados, erros, calibração Hall e todos
os parâmetros do perfil. A leitura USB da placa real identificou
`fw_version_unreleased=1`, além de 0.5.1; esse valor está explicitamente no
perfil esperado. Nenhum ganho, limite ou parâmetro elétrico é regravado para
contornar uma divergência. `safety_profile` permite indicar outro arquivo
quando houver comissionamento documentado de outra placa.

No engate, a referência e a integral são zeradas, o watchdog de 2 s é
habilitado/alimentado e CLOSED_LOOP precisa ser confirmado nos dois eixos.
Na liberação, o driver tenta zero e IDLE em ambos mesmo se uma escrita falhar.
O watchdog só é desligado depois de confirmar IDLE; se isso falhar, permanece
habilitado para expirar. Uma falha exige reconexão explícita e não limpa erros.

Conexão e espera de calibração/índice têm prazos configuráveis. O comando
expira após `command_timeout_sec` (0,5 s). A liberação por ociosidade usa
`idle_release_sec` (2 s), em vez de depender de 100 execuções do timer.
No hardware, uma janela de 0,3 s com referência interna zerada e posição
contida pode antecipar a liberação. O timer de supervisão usa relógio
monotônico, independente do relógio ROS.
O perfil solicita 20 Hz de supervisão/odometria no host, sem alterar o laço
de controle interno da placa.
Comandos não finitos são rejeitados, interrompem o comando anterior e não
renovam o prazo. Uma falha de leitura/comando encerra a interface; não há
tentativa automática de reconectar e voltar a mover.

Durante a sessão, as guardas verificam erros, estado, watchdog, tensão
entre 30 e 50 V, Iq medido até 2,4 A, temperatura do inversor até 60 °C,
velocidade estimada até 115% do limite e aquisição/intervalo de telemetria
até 0,5 s. A suspeita de roda presa considera referência interna, velocidade
e corrente por pelo menos 0,1 s. Esses limiares vêm do ensaio de calibração;
não representam caracterização térmica ou validação de navegação.

As referências das rodas são escaladas juntas no limite de 0,931820 volta/s,
preservando sua proporção. A rampa de 0,75 volta/s² permanece no firmware.
Em reta, a correção externa só reduz a roda adiantada: ganho de 1/s e máximo
de 0,08 volta/s. Em curvas não há correção para igualar os percursos.
A guarda de 2,5 cm compara a diferença entre resíduos de percurso medido e
esperado pela referência interna de cada eixo. Sua generalização para curvas
passou nos testes de lógica e ainda precisa de validação física.

O limite de excursão de 1,5 m do ensaio original pertence àquela missão de
ida/volta, não à navegação contínua. Cada executor de ensaio físico define
seu próprio limite de percurso e duração.

## Serviços

São serviços `std_srvs/srv/Trigger`:

| Serviço | Resultado |
|---|---|
| `/odrive_node/connect` | Conecta, sem engatar ou calibrar implicitamente |
| `/odrive_node/disconnect` | Para, libera e desconecta |
| `/odrive_node/engage` | Engata uma interface conectada |
| `/odrive_node/release` | Para e libera, mantendo a conexão |
| `/odrive_node/calibrate` | Calibra uma interface conectada e liberada |
| `/reset_odometry` | Zera a pose, mantendo a referência atual dos encoders |

Os nomes privados acompanham o nome/namespace do nó. O reset preserva o
nome público anterior. As respostas preenchem `success` e `message`.
Esses serviços atuam na placa quando o backend físico está selecionado.
Uma falha nesses serviços também encerra a sessão; por exemplo, solicitar
calibração com os motores engatados resulta em falha e parada, exigindo
reconexão explícita. Uma falha de leitura durante reset não é ignorada.

## Odometrias disponíveis

| Parâmetro | Papel |
|---|---|
| `odometry_mode=legacy` | Mantém a integração e a escala de velocidade anteriores |
| `legacy_velocity_scale=100.0` | Explicita o antigo fator fixo, usado somente em `legacy` |
| `odometry_mode=encoder` | Usa unidades declaradas e integra arcos sem aproximação lateral legada |
| `encoder_velocity_units=turns_per_second` | Unidade da API 0.5.x |
| `encoder_velocity_units=counts_per_second` | Conversão explícita para fontes que usam contagens/s |
| `encoder_yaw_sign=1.0` | Padrão ROS com esquerda/direita corretamente identificadas |
| `encoder_yaw_sign=-1.0` | Comparação com o mapeamento histórico |

Na [interface oficial 0.5.1](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.1/Firmware/odrive-interface.yaml),
`encoder.vel_estimate` é voltas/s e `pos_cpr_counts` é contagens no intervalo
de uma revolução. O código anterior misturava essas unidades e usava sinais
opostos para pose e velocidade angular. O modo `encoder` usa um mesmo sinal
e converte velocidade sem o fator empírico. O firmware instalado foi
identificado como 0.5.1; sinais, distâncias e giros ainda exigem comparação
física com uma referência externa.

O perfil físico agora seleciona `encoder`. `legacy` continua disponível
para comparação histórica, com seu fator empírico e incompatibilidade angular.
A mudança mantém a polaridade de avanço e normaliza o comando angular à
convenção ROS. O sinal angular
físico permanece pendente; giros estão bloqueados por padrão.

A primeira leitura estabelece a referência, evitando deslocamento fictício
na inicialização. Reconexão e retorno do relógio estabelecem nova referência
sem alterar a pose acumulada. A integração continua quando a publicação de
odometria está desligada. A diferença de encoder circular pressupõe movimento
inferior a meia revolução por leitura; falhas prolongadas exigem reconexão,
sem integrar o trecho perdido como se tivesse sido observado.

## Publicações e configuração

`publish_odom`, `publish_odom_tf`, `publish_raw_odom` e `publish_current`
controlam suas saídas independentemente. TF pode ser publicada com odometria
desativada. Image/CameraInfo e sensores continuam sendo responsabilidade
dos outros pacotes.

As configurações do driver são somente de leitura depois da inicialização.
Para alterá-las, edite o YAML e reinicie o nó. Isso evita aceitar mudanças
que ficariam apenas no servidor de parâmetros. Os parâmetros legados
`publish_joint_angles` e `publish_temperatures` ainda não são suportados;
quando solicitados, geram aviso. Não são publicadas medidas inventadas.

## Testar sem placa

```bash
make test
```

O comando compila os oito pacotes de simulação e executa os testes públicos,
incluindo mensagens e serviços ROS com uma interface ODrive simulada. Não
requer simulação iniciada, acesso USB ou a pasta `extras/`.
Os testes adicionais de matemática, relógio e firmware falso ficam no
acervo local: `make test-private`. O alvo local `make odrive-check` executa
os testes ROS em um container de simulação já iniciado, no domínio 29.
Esses verificadores não enviam comandos ao robô físico.

Para experimentar manualmente dentro do container, em um terminal separado:

```bash
ROS_DOMAIN_ID=29 ros2 launch robis_odrive odrive_launch.py \
  simulation_mode:=true odometry_mode:=encoder
```

Os demais terminais usados para observar ou comandar esse teste também devem
usar domínio 29. Evite iniciar esse driver no domínio 28 da simulação: ele
duplicaria a odometria e TF do DiffDrive. Para enviar uma velocidade ou chamar
um serviço, use os comandos ROS habituais nesse mesmo domínio.
