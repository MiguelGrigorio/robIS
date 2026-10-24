# Jetson Nano e ROS 2 Jazzy

Hardware confirmado: Nano de 4 GB, JetPack 4.6.6, L4T 32.7.6.

Robô, desenvolvimento e simulação usam ROS 2 Jazzy. Isso mantém a mesma API,
versão de Python e família de dependências em todos os perfis. Jazzy tem suporte
até maio de 2029 e é a distribuição já usada com Gazebo Harmonic no projeto.
Referências: [REP-2000](https://reps.openrobotics.org/rep-2000/) e
[Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/).

## Sistema da Nano

A Nano mantém o JetPack instalado. O L4T usa kernel 4.9 e Ubuntu 18.04,
conforme a [NVIDIA](https://developer.nvidia.com/embedded/linux-tegra-r3276).
O container fornece Ubuntu 24.04 e Jazzy para os programas do robô, mas continua
usando o kernel do host. Em 22/10/2026, a imagem básica foi compilada e
testada na Nano com kernel 4.9.337-tegra: C920, X4, TFs e ODrive em IDLE
funcionaram juntos. Veja os [resultados e limites do teste](hardware.md).

O funcionamento básico usa CPU: controle de motores, odometria, câmera e lidar.
O SDK do YDLidar e a API do ODrive são instalados durante o build da imagem.
A imagem final não precisa montar código nem compilar na inicialização.

## GPU em uma próxima etapa

A inferência acelerada precisará de bibliotecas CUDA/TensorRT compatíveis com
L4T 32.7.6. O perfil atual não injeta essas bibliotecas no container Ubuntu 24.04
e não anuncia suporte à GPU da Nano.

Essa etapa será preparada como um serviço L4T de inferência, com um contrato de
entrada/saída que possa ser consumido pelos nós Jazzy. O serviço poderá executar
sem instalar ROS. O ambiente ROS do robô continuará unificado.

A GPU é uma etapa posterior; o ensaio físico atual valida o perfil básico CPU.
