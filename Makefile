.DEFAULT_GOAL := help
ROBOT_IMAGE ?= robis:jazzy-basic
ROBOT_PLATFORM ?= linux/arm64
JETSON ?= jetson@10.10.1.220
NAVIGATION_IMAGE ?= robis:jazzy-navigation
SEMANTIC_BASE ?= robis:jazzy-semantic-base
SEMANTIC_WEIGHTS ?= config/models/labsea-yolo26s-seg.pt
SIM_IMAGE ?= robis:jazzy-sim
SEMANTIC_TORCH_INDEX ?= https://download.pytorch.org/whl/cu130
SEMANTIC_SIM_IMAGE ?= robis:jazzy-semantic-sim
SEMANTIC_COMPOSE = docker compose --project-directory . -p robis_semantic_manual -f docker/compose/semantic.yml

.PHONY: help sim-down deploy deploy-navigation dev-build dev-logs dev-shell dev-up down robot-build robot-down robot-navigation-build robot-navigation-down robot-navigation-up robot-save robot-up sim-build sim-gui sim-logs sim-nav sim-semantic-build sim-semantic-down sim-semantic-logs sim-semantic-shell sim-semantic-stop sim-semantic-up sim-shell sim-slam sim-up test

help:
	@echo "Simulação:  make sim-up | sim-shell | sim-down"
	@echo "Verificar:  make test | sim-check"
	@echo "LabSEA GPU: make sim-semantic-base-build | sim-semantic-build | sim-semantic-up"
	@echo "Robô:       make robot-build | deploy (destino: $(JETSON))"
	@echo "Código:     make dev-up | dev-shell | dev-build"
	@echo "Desligar:   make down"
	@echo "Ferramentas locais de pesquisa/teste: extras/"

sim-down:
	docker compose --project-directory . -f docker/compose/development.yml --profile sim down

.PHONY: sim-semantic-base-build
sim-semantic-base-build:
	@test -f "$(SEMANTIC_WEIGHTS)" || (echo "Informe o checkpoint LabSEA: SEMANTIC_WEIGHTS=/caminho/best.pt"; exit 2)
	docker build -f docker/Dockerfile.sim -t robis:jazzy-sim .
	docker build -f docker/Dockerfile.detector \
		--build-arg TORCH_INDEX_URL=$(SEMANTIC_TORCH_INDEX) \
		-t robis:jazzy-detector-runtime .
	docker build -f docker/Dockerfile.semantic-base \
		--build-arg SIM_MODEL_FILENAME="$(notdir $(SEMANTIC_WEIGHTS))" \
		--build-context detector_weights="$(dir $(abspath $(SEMANTIC_WEIGHTS)))" \
		-t $(SEMANTIC_BASE) .

sim-semantic-build:
	docker build -f docker/Dockerfile.semantic --build-arg EXPERIMENT_BASE=$(SEMANTIC_BASE) -t $(SEMANTIC_SIM_IMAGE) .

sim-semantic-up:
	ROBIS_DATA_OWNER=$(shell id -u):$(shell id -g) ROBIS_SEMANTIC_SIM_IMAGE=$(SEMANTIC_SIM_IMAGE) $(SEMANTIC_COMPOSE) up -d --no-build --wait --wait-timeout 240

sim-semantic-down:
	$(SEMANTIC_COMPOSE) down

sim-semantic-shell:
	$(SEMANTIC_COMPOSE) exec semantic /usr/local/bin/robis-entrypoint bash

sim-semantic-logs:
	$(SEMANTIC_COMPOSE) logs -f semantic

sim-semantic-stop:
	$(SEMANTIC_COMPOSE) exec -T semantic /usr/local/bin/robis-entrypoint ros2 run robis_semantic_navigation stop

test:
	docker build -f docker/Dockerfile.sim -t $(SIM_IMAGE) .
	docker run --rm --runtime runc --network none --cpus 2 --memory 3g \
		-e ROS_DOMAIN_ID=79 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
		-v "$(CURDIR):/source:ro" $(SIM_IMAGE) bash -c \
		'cp -a /source/src /source/config /source/scripts /robis_ws/ && exec bash /robis_ws/scripts/test_simulation.sh'

.PHONY: sim-check
sim-check:
	docker compose --project-directory . -f docker/compose/development.yml exec -T -e ROBIS_BUILD_ON_START=false robis_sim /usr/local/bin/robis-entrypoint python3 -m robis_gazebo.check_simulation --move

robot-build:
	docker buildx build --platform $(ROBOT_PLATFORM) --target robot -f docker/Dockerfile.robot -t $(ROBOT_IMAGE) --load .

robot-navigation-build:
	docker buildx build --platform $(ROBOT_PLATFORM) --target robot-navigation -f docker/Dockerfile.robot -t $(NAVIGATION_IMAGE) --load .

robot-navigation-up:
	ROBIS_NAVIGATION_IMAGE=$(NAVIGATION_IMAGE) docker compose --project-directory . -f docker/compose/robot.yml -f docker/compose/robot-navigation.yml up -d

robot-navigation-down:
	docker compose --project-directory . -f docker/compose/robot.yml -f docker/compose/robot-navigation.yml down

robot-save:
	docker save -o robis-jazzy-basic.tar $(ROBOT_IMAGE)

robot-up:
	ROBIS_ROBOT_IMAGE=$(ROBOT_IMAGE) docker compose --project-directory . -f docker/compose/robot.yml up -d

robot-down:
	docker compose --project-directory . -f docker/compose/robot.yml down

dev-up:
	docker compose --project-directory . -f docker/compose/development.yml --profile dev up -d --build robis_dev

sim-up:
	docker compose --project-directory . -f docker/compose/development.yml --profile sim up -d --build robis_sim

dev-shell:
	docker compose --project-directory . -f docker/compose/development.yml exec -e ROBIS_BUILD_ON_START=false robis_dev /usr/local/bin/robis-entrypoint bash

sim-shell:
	docker compose --project-directory . -f docker/compose/development.yml exec -e ROBIS_BUILD_ON_START=false robis_sim /usr/local/bin/robis-entrypoint bash

dev-build:
	docker compose --project-directory . -f docker/compose/development.yml exec robis_dev /robis_ws/scripts/build_workspace.sh basic --symlink

sim-build:
	docker compose --project-directory . -f docker/compose/development.yml exec robis_sim /robis_ws/scripts/build_workspace.sh simulation --symlink

sim-slam:
	ROBIS_SIM_STACK=slam docker compose --project-directory . -f docker/compose/development.yml --profile sim up -d --build robis_sim

sim-nav:
	ROBIS_SIM_STACK=navigation docker compose --project-directory . -f docker/compose/development.yml --profile sim up -d --build robis_sim

sim-gui:
	ROBIS_GZ_ARGS="-r -v 2" docker compose --project-directory . -f docker/compose/development.yml -f docker/compose/simulation-gui.yml --profile sim up -d robis_sim

down:
	docker compose --project-directory . -f docker/compose/development.yml --profile dev --profile sim down

deploy:
	bash scripts/deploy_robot.sh "$(JETSON)" "$(ROBOT_IMAGE)"

deploy-navigation:
	bash scripts/deploy_robot.sh "$(JETSON)" "$(NAVIGATION_IMAGE)" navigation

dev-logs:
	docker compose --project-directory . -f docker/compose/development.yml logs -f robis_dev

sim-logs:
	docker compose --project-directory . -f docker/compose/development.yml logs -f robis_sim


# Optional private tools; operating the robot does not depend on extras.
-include extras/Makefile
