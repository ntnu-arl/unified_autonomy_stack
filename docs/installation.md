# Installation

### This page details the instructions to install the Unified Autonomy Stack. 

The instructions explain how to setup the entire stack to use in your work or run the provided demos. Further details about the structuring of the stack can be found on the [Deployment](deployment.md) page.

## Requirements

### Computer requirements

The stack has been tested on Ubuntu 20.04 and 22.04. As the stack includes learning-based navigation modules, an Nvidia GPU is required to run those.

### Docker

The stack is organized as a collection of docker containers, hence, first install docker using the instructions on the [official webpage](https://docs.docker.com/engine/install/ubuntu/).

#### Docker configuration

Configure the `/etc/docker/daemon.json` as follows:

```bash
{
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    }
}
```

**Note:** Do not use "nvidia" as the default run time as this may cause issues with some of the packages.

Add the following line to `~/.bashrc` or `~/.zshrc`:

```bash
xhost +local:docker
```

### Install other dependencies

```bash
sudo apt install python3-vcstool
```

## Unified Autonomy Stack Installation

### Clone the required packages

Clone the base repository:

```bash
git clone git@github.com:ntnu-arl/unified_autonomy_stack.git
```

Clone individual packages:


```bash
cd unified_autonomy_stack
mkdir workspaces
./scripts/import_all_repos.sh  # recursively clones all the repositories
```

_Note: Cloning all repositories will take some time (especially ws_sim.repos), please be patient_

### Generate docker images

```bash
# cd unified_autonomy_stack
make images
```

_Note: Building the images for the firs time will take some time, please be patient_

### Build the code

#### There are two ways to build the code:  

**Option1:** Build all workspaces in parallel. This method is faster but it can be tedious to see the output of individual workspaces.

```bash
# cd unified_autonomy_stack
make build
```

**Option2:** Build one workspace at a time. This method is slower but easier to track the output of each workspace.

```bash
# cd unified_autonomy_stack
make build-sequential
```

_Note: If you are building the stack on a low ram computer, it is advised to use **Option2** as it will not make the ram fill up causing the build to fail._

## Configure ROS_DOMAIN_ID

Set the variable `DOMAIN_ID` in the `unified_autonomy_stack/.env` file
