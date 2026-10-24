# Containers

Os comandos públicos ficam no Makefile da raiz. Todos os builds usam a raiz
como contexto, por exemplo `docker build -f docker/Dockerfile.robot .`.

| Arquivo | Finalidade |
| --- | --- |
| `Dockerfile.robot` | Imagens básica e completa do robô, com SDKs e pacotes compilados |
| `Dockerfile.sim` | Desenvolvimento ROS/Gazebo com código compartilhado |
| `Dockerfile.detector` | Runtime opcional do detector aprendido |
| `Dockerfile.semantic-base` | Incorpora o checkpoint LabSEA ao runtime do detector |
| `Dockerfile.semantic` | Aplicação de simulação compilada; alvo `experiments` acrescenta ferramentas locais |
| `compose/development.yml` | Serviços `dev` e `sim` |
| `compose/simulation-gui.yml` | Janela local opcional; adiciona autenticação X11 |
| `compose/robot.yml` | Robô básico |
| `compose/robot-navigation.yml` | Complemento de navegação do robô |
| `compose/semantic.yml` | LabSEA semântico sem janela na Orin |

Para invocar Compose diretamente, mantenha a raiz como diretório do projeto:

```bash
docker compose --project-directory . -f docker/compose/development.yml \
  --profile sim config
```

As variantes de coleta, treino, benchmark, VLM e vídeo ficam em `extras/`.
O alvo padrão de `Dockerfile.semantic` não copia esses experimentos.
