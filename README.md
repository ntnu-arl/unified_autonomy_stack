# Unified Autonomy Stack

Warning: This repository is under active development and will be updated frequently.

## Setup

- Install Docker Engine on your machine. Follow the instructions for your system [from Docker's official documentation](https://docs.docker.com/engine/install/). Make sure that you do the [linux post-install](https://docs.docker.com/engine/install/linux-postinstall) steps to run docker as a non-root user.

> **_NOTE:_**  For Orin NX on Jetpack 6.X there exists a known issue with Docker Engine as mentioned [here](https://forums.developer.nvidia.com/t/iptables-error-message/333007). The workaround is to do: `sudo apt-get install -y docker-ce=5:27.5* docker-ce-cli=5:27.5* --allow-downgrades && sudo apt-mark hold docker-ce docker-ce-cli`

- Setup the NVIDIA Container Toolkit if you have an NVIDIA GPU. Follow the instructions for your system [from NVIDIA's official documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). For Ubuntu based systems you can follow the steps at <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian>. Then you need to configure the Docker Daemon as per instructions <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker> . You do not need Rootless mode for this stack.

- Get this repository:

  ```bash
  git clone git@github.com:ntnu-arl/unified_autonomy_stack.git
  cd unified_autonomy_stack
  ```

- Install [`vcstool`](https://github.com/dirk-thomas/vcstool/)
- Make the scripts executable and import repositories:

  ```bash
  chmod +x ./scripts/*.sh
  ./scripts/import_all_repos.sh
  ```

## Development Workflow

The stack uses a Makefile to manage common development tasks. To see all available commands:

```bash
make help
```

### Quick Start

```bash
make images     # Build Docker images - Will take a long time the first time
make build      # Build all code workspaces - Will take a long time the first time
make launch     # Launch the stack
```

## Adding new packages

To add your own package to the stack:

1. Create a new `ws_<your_package_name>` folder in the `workspaces` directory
2. Add your package and dependencies to `ws_<your_package_name>/src`
3. Add build and launch services to `docker-compose.yml`
4. If you need a different dockerfile, create it and reference it in `docker-bake.hcl`

## Version Control

Use vcs to manage repository versions. After making changes:

```bash
./scripts/export_all_repos.sh
```

This exports exact versions of all repositories to track the working state of the stack.

## Cite

If you use this code in your research, please cite:

<TODO>

## Contact

For questions, please open an issue or contact:

- [Nikhil Khedekar](mailto:nikhil.v.khedekar@ntnu.no)
- [Mihir Dharmadhikari](mailto:mihir.dharmadhikari@ntnu.no)

<TODO>
