# Build e deploy

Execute os comandos na raiz deste repositório.

Este guia descreve a imagem básica (`--target robot`). Para o perfil completo,
`make robot-navigation-build` gera `--target robot-navigation`, com Nav2,
semântica e detector CPU já instalados. `make deploy-navigation` transfere
essa imagem. Veja [a configuração física completa](real-navigation.md).

## O que cada imagem contém

`docker/Dockerfile.robot` tem etapas separadas. A etapa de desenvolvimento instala
os compiladores e compila o SDK do YDLidar. A etapa builder compila apenas os
seis pacotes do perfil básico, sem symlinks para o código-fonte.
A etapa final copia o SDK instalado e os pacotes instalados.

O build baixa o SDK de [YDLIDAR/YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK),
fixado no commit `7c0a2328fd85b65960c37287a67b0edbee8caa4b`, idêntico à cópia
usada nos ensaios anteriores. O clone e os compiladores ficam na etapa de
build; o robô recebe a biblioteca instalada. A primeira construção precisa
de internet. `YDLIDAR_SDK_COMMIT` permite escolher outra revisão explicitamente.

A imagem básica final não contém `src/`, ferramentas adicionadas para o build do SDK,
Gazebo, Nav2 ou os pacotes de navegação semântica. A imagem base ROS pode trazer
suas próprias ferramentas. A API do ODrive e suas dependências Python são
instaladas durante o build, em `/opt/odrive-python`.

A API usada é `odrive==0.6.11.post0`, a mesma versão da imagem usada nos
ensaios USB/ROS da Nano em 22/10/2026. Isso é a versão da biblioteca Python;
o firmware da placa permanece 0.5.1.
Pode ser alterada no build com `--build-arg ODRIVE_VERSION=...`.

## Gerar a imagem

Na própria Nano, com Docker disponível, o build nativo também pode ser feito
sem Buildx:

```bash
docker build --target robot -f docker/Dockerfile.robot -t robis:jazzy-basic .
```

Compilação e SDK continuam dentro do build da imagem. O script limita a
compilação a um worker para caber nos 4 GB; o início do container já usa os
pacotes instalados. Para compilar em outro computador para ARM64, use os
atalhos com Buildx abaixo.

```bash
make robot-build
```

O padrão é `linux/arm64` e o nome `robis:jazzy-basic`.
No PC, Docker Buildx precisa ter um builder ARM64 ou emulação ARM64 previamente
configurada. Não é necessário instalar ROS, o SDK ou o ODrive no host.

No Docker Desktop, a emulação normalmente já está disponível.
No Docker Engine Linux, uma opção é configurar QEMU uma vez:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
docker buildx inspect --bootstrap
```

Esse comando altera o registro de emulação do host. Outra opção é usar um
builder ARM64 remoto ou compilar na própria Nano. O build usa um worker CMake
e execução sequencial para reduzir a pressão de memória na Nano.

O build verifica a importação da API ODrive, a instalação do SDK, a existência
do driver compilado, o vínculo com a biblioteca do lidar e os argumentos do
launch básico. Um build bem-sucedido ainda não comprova funcionamento físico.

No perfil completo, os oito pacotes são compilados diretamente a partir das
fontes comuns. As dependências de navegação têm sua própria etapa de runtime,
sem uma segunda cópia da instalação básica. São instalados os componentes
Nav2 usados pelo robIS; os metapacotes que trazem simulações de outros robôs
ficam no ambiente de simulação.

## Transferir sem levar o código

```bash
make deploy
```

O destino padrão é `jetson@10.10.1.220`. O SSH solicita a senha no terminal
ou utiliza a chave já configurada. Para outro destino, use
`make deploy JETSON=usuario@host`; um alias SSH como `JETSON=robis` também funciona.

O script valida que a imagem é ARM64, envia `docker save` comprimido por SSH,
carrega a imagem no destino e copia apenas o pequeno script de início.
Ele não instala pacotes no host e não inicia o hardware.

Se preferir transferência manual:

```bash
make robot-save
scp robis-jazzy-basic.tar jetson@10.10.1.220:
ssh jetson@10.10.1.220 'docker load -i robis-jazzy-basic.tar'
ssh jetson@10.10.1.220 'mkdir -p ~/robis'
scp scripts/run_robot.sh jetson@10.10.1.220:robis/run_robot.sh
ssh jetson@10.10.1.220 'chmod +x ~/robis/run_robot.sh'
```

## Iniciar depois

Na Nano:

```bash
~/robis/run_robot.sh
docker logs -f robis
docker stop robis
```

Depois do deploy, `~/robis/run_robot.sh` cria e inicia o container básico,
sem compilar e sem montar o código. Ele inicia motores, odometria, câmera,
LiDAR e TFs. A ODrive conecta em IDLE; movimento depende dos comandos ROS.
Nav2 e memória semântica pertencem ao perfil completo, que ainda precisa
das calibrações e validações físicas descritas no seu guia.

Após `docker stop robis`, use `docker start robis` para iniciar novamente.
O script recusa criar outro container com o mesmo nome; não substitui uma
instalação existente. Após atualizar a imagem, pare e remova o container
antigo com `docker stop robis` e `docker rm robis`, e execute o script outra vez.

Como alternativa, com Docker Compose disponível:
`make robot-up` usa `docker/compose/robot.yml`, sem volume de código.

A execução expõe os dispositivos USB/serial e usa rede do host. O runtime
`runc` evita injetar bibliotecas CUDA do L4T no ambiente Ubuntu 24.04.
Os paths iniciais são `/dev/ttyUSB0` e `/dev/video0`, confirmados no ensaio
básico da Nano em outubro/2026; podem mudar se os dispositivos forem reconectados.

Para trocar a imagem de um container já criado, pare e remova esse container
antes de iniciar o novo. Os scripts não substituem automaticamente um robô em execução.

## Desenvolvimento

O Compose usa as etapas de dependências e compartilha `src/`.
Todos os perfis usam ROS 2 Jazzy sobre Ubuntu 24.04. Build, install e logs
ficam em volumes separados por perfil, pois os conjuntos de pacotes diferem.
Os volumes possuem o prefixo jazzy para não reutilizar builds anteriores.
O build de desenvolvimento é feito ao subir o serviço; o build de produção é feito ao gerar a imagem.

- `make dev-up` / `make dev-shell`: perfil básico Jazzy.
- `make sim-up` / `make sim-shell`: Gazebo/Jazzy no computador.
- `make dev-build` / `make sim-build`: recompilar após mudar C++, arquivos novos
  ou metadados de instalação.
- `make down`: parar os dois serviços.

Mudanças em dependências do sistema exigem reconstruir a imagem.
Os domínios ROS são separados: robô/desenvolvimento 27 e simulação 28.
Os domínios separados evitam misturar a simulação com os tópicos do robô.
Para visualizar o robô no PC, use Jazzy e o domínio 27.
