# make setup
# make deploy
JETSON_USER := jetson
JETSON_IP   := 10.10.1.220
JETSON_SSH  := $(JETSON_USER)@$(JETSON_IP)
IMAGE_NAME  := robis
USER_NAME   := miguelgrigorio27/
IMAGE_TAG   := jazzy
DOCKERFILE  := Dockerfile.robot
SIM_DOCKERFILE := Dockerfile.sim
TAR_FILE    := $(IMAGE_NAME).tar

.PHONY: help setup build transfer load deploy clean compose-build sim-up sim-shell sim-logs sim-down build-sim run-sim run-robot

# ─────────────────────────────────────────────
help:
	@echo ""
	@echo "  make setup      Instala QEMU e configura buildx para cross-compile"
	@echo "  make build      Compila imagem arm64 no PC (robot)"
	@echo "  make compose-build Compila a imagem da simulação via Compose"
	@echo "  make sim-up      Sobe a simulação em segundo plano"
	@echo "  make sim-shell   Abre um shell no container da simulação"
	@echo "  make sim-logs    Mostra os logs do container da simulação"
	@echo "  make sim-down    Para a simulação"
	@echo "  make transfer   Envia imagem para a Jetson via scp"
	@echo "  make load       Carrega imagem na Jetson"
	@echo "  make deploy     build + transfer + load (fluxo completo)"
	@echo "  make run-sim    Executa o container de simulação localmente"
	@echo "  make run-robot  Executa o container do robô na Jetson"
	@echo "  make clean      Remove o .tar local"
	@echo ""

# ─────────────────────────────────────────────
setup:
	@echo "[setup] Instalando QEMU..."
	sudo apt-get install -y qemu-user-static
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
	docker buildx create --name jetson-builder --use || docker buildx use jetson-builder
	docker buildx inspect --bootstrap
	@echo "[setup] Pronto! Cross-compile arm64 habilitado."

# ─────────────────────────────────────────────
build:
	@echo "[build] Compilando imagem $(IMAGE_NAME):$(IMAGE_TAG) para linux/arm64..."
	docker buildx build \
		--platform linux/arm64 \
		-f $(DOCKERFILE) \
		-t $(USER_NAME)$(IMAGE_NAME):$(IMAGE_TAG) \
		--load \
		.
	@echo "[build] Salvando imagem em $(TAR_FILE)..."
	docker save $(USER_NAME)$(IMAGE_NAME):$(IMAGE_TAG) -o $(TAR_FILE)
	@echo "[build] Imagem salva em $(TAR_FILE) ($$(du -sh $(TAR_FILE) | cut -f1))"

# ─────────────────────────────────────────────
compose-build:
	docker compose build robis_sim

sim-up:
	docker compose up -d robis_sim

sim-shell:
	docker compose exec robis_sim bash

sim-logs:
	docker compose logs -f robis_sim

sim-down:
	docker compose down

build-sim: compose-build

# ─────────────────────────────────────────────
transfer:
	@echo "[transfer] Enviando $(TAR_FILE) para $(JETSON_SSH)..."
	scp $(TAR_FILE) $(JETSON_SSH):~/$(TAR_FILE)
	@echo "[transfer] Concluído."

# ─────────────────────────────────────────────
load:
	@echo "[load] Carregando imagem na Jetson..."
	ssh $(JETSON_SSH) "docker load -i ~/$(TAR_FILE)"
	@echo "[load] Imagem carregada na Jetson."

# ─────────────────────────────────────────────
deploy: build transfer load
	@echo ""
	@echo "✅ Deploy completo! Para rodar na Jetson:"
	@echo ""
	@echo "  ssh $(JETSON_SSH)"
	@echo "  sudo docker run --runtime nvidia -it --rm \\"
	@echo "    --network=host --privileged -v /dev:/dev \\"
	@echo "    $(USER_NAME)$(IMAGE_NAME):$(IMAGE_TAG) \\"
	@echo "    ros2 launch robis_bringup real_full.launch.py stack:=none"
	@echo ""

# ─────────────────────────────────────────────
clean:
	@echo "[clean] Removendo $(TAR_FILE)..."
	rm -f $(TAR_FILE)
	@echo "[clean] Pronto."

run-sim: sim-up

run-robot:
	ssh $(JETSON_SSH) "docker run -it --rm \
	--network=host \
	--privileged \
	-v /dev:/dev \
	-e ROS_DOMAIN_ID=0 \
	-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
	miguelgrigorio27/robis:jazzy \
	/bin/bash"
