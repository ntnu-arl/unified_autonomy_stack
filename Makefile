# Convenience wrapper for docker-compose operations
# ==================== HELP ====================
.PHONY: help
.DEFAULT_GOAL := help

help: ## Show this help message
	@echo "Available make targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'
	@echo "Build services in docker-compose.yml:"
	@echo "  $(BUILD_SERVICES_STRIPPED)"
	@echo "Launch services in docker-compose.yml:"
	@echo "  $(LAUNCH_SERVICES_STRIPPED)"
	@echo ""

# ==================== IMAGE BUILDING ====================
.PHONY: images

images: ## Build all Docker images
	@docker buildx bake

# ==================== CODE BUILDING ====================
.PHONY: build build-% build-sequential

# Derive list of the "build_" services from docker-compose services
DOCKER_COMPOSE_FILE := docker-compose.yml
DOCKER_COMPOSE_BUILD_FILE := docker-compose.build.yml
BUILD_SERVICES_STRIPPED := $(shell docker compose -f $(DOCKER_COMPOSE_BUILD_FILE) --profile build config --services 2>/dev/null | awk -F'build_' '/^build_/{print $$2}')

build: ## Build all services in parallel (messy output)
	docker compose -f $(DOCKER_COMPOSE_BUILD_FILE) --profile build up --remove-orphans;

build-%: ## Pattern rule for building specific services
	@docker compose -f $(DOCKER_COMPOSE_BUILD_FILE) --profile build up --remove-orphans build_$*

build-sequential: ## Build all services sequentially (one-by-one and slower, but cleaner output)
	@for pkg in $(BUILD_SERVICES_STRIPPED); do \
		$(MAKE) build-$$pkg || exit 1; \
	done


# ==================== SERVICE MANAGEMENT ====================
.PHONY: launch stop restart

LAUNCH_SERVICES_ROS1 := $(shell docker compose -f $(DOCKER_COMPOSE_FILE) --profile launch config --services 2>/dev/null | grep '^ros1_launch_' | sed 's/^ros1_launch_//')
LAUNCH_SERVICES_ROS2 := $(shell docker compose -f $(DOCKER_COMPOSE_FILE) --profile launch config --services 2>/dev/null | grep '^ros2_launch_' | sed 's/^ros2_launch_//')
LAUNCH_SERVICES_STRIPPED := $(LAUNCH_SERVICES_ROS1) $(LAUNCH_SERVICES_ROS2)

launch: ## Launch all services
	@docker compose -f $(DOCKER_COMPOSE_FILE) --profile launch up

#TODO: launch for specific services

stop: ## Stop all launched services
	@docker compose --profile launch down

restart: ## Restart all launched services
	@docker compose --profile launch restart

# ==================== MONITORING & DEBUG ====================
.PHONY: status status-all logs

status:
	@docker compose --profile launch ps

status-all: ## All Services Status (all profiles)
	@docker compose --profile images --profile build --profile runtime ps

logs: ## show logs from all services
	@docker compose --profile launch logs -f

# TODO: Pattern rule for opening shell in specific services

# ==================== CLEANUP ====================
.PHONY: clean clean-all

clean:
	@docker compose --profile launch --profile build down

clean-all:
	@docker compose --profile launch --profile build down -v
