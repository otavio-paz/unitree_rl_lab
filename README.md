# RL Pipeline for G1-29dof

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.5.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.1.1-silver)](https://isaac-sim.github.io/IsaacLab)
[![Unitree RL Lab](https://img.shields.io/badge/unitree_rl_lab-1.0.0-silver.svg)](https://github.com/unitreerobotics/unitree_rl_lab)
[![MuJoCo](https://img.shields.io/badge/mujoco-v3.2.7-silver)](https://mujoco.readthedocs.io/en/stable/overview.html)
[![Unitree_Mujoco](https://img.shields.io/badge/unitree_mujoco-1.0.0-silver)](https://github.com/unitreerobotics/unitree_mujoco)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg)](https://opensource.org/license/apache-2-0)

## Table of Contents
1. [Overview](#overview)
    * [Process](#-process)
3. [Installation](#-installation)
4. [Pipeline](#-pipeline)
    * [Training policy](#training-policy)
    * [Testing policy](#testing-policy)
    * [Sim2Sim](#sim2sim)
    * [Sim2Real](#sim2real)
9. [Contributions](#-contributions)
10. [Troubleshooting](#-troubleshooting)
11. [Acknowledgements](#acknowledgements)

## Overview

This project describes the pipeline for reinforcement learning on Unitree G1 (29 dof) tested at [Kantaros Lab](https://sites.wustl.edu/kantaroslab/) in Washington University in St. Louis and it's based on the repository [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab), which provides a set of reinforcement learning environments for Unitree robots and it's built on top of [IsaacLab](https://github.com/isaac-sim/IsaacLab). Currently, the `unitree_rl_lab` supports Unitree **Go2**, **H1** and **G1-29dof** robots.

As part of this project, I was recommended to install a dual boot of Windows 11 and Ubuntu 22.04.5 LTS, which it was done by the main author and the steps taken have been documented in ``.

<div align="center">

| <div align="center"> Isaac Lab </div> | <div align="center">  Mujoco </div> |  <div align="center"> Physical </div> |
|--- | --- | --- |
| [<img src="https://oss-global-cdn.unitree.com/static/d879adac250648c587d3681e90658b49_480x397.gif" width="240px">](g1_sim.gif) | [<img src="https://oss-global-cdn.unitree.com/static/3c88e045ab124c3ab9c761a99cb5e71f_480x397.gif" width="240px">](g1_mujoco.gif) | [<img src="https://oss-global-cdn.unitree.com/static/6c17c6cf52ec4e26bbfab1fbf591adb2_480x270.gif" width="240px">](g1_real.gif) |

</div>

### 🔁 Process

The basic workflow for using reinforcement learning to achieve motion control is:

`Train` → `Play` → `Sim2Sim` → `Sim2Real`

- **Train**: Use of Isaac Lab simulation environment to let the robot interact with the environment and find a policy that maximizes the designed rewards. Real-time visualization during training is not recommended to avoid reduced efficiency.
- **Play**: Use the Play command to verify the trained policy and ensure it meets expectations.
- **Sim2Sim**: Deploy the Lab-trained policy to other simulators to ensure it’s not overly specific to Isaac Lab characteristics.
- **Sim2Real**: Deploy the policy to a physical robot to achieve motion control.

## Installation

- Install NVIDIA Driver
  - Isaac Sim/Lab has certain [requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) for versions of the drivers. For the tests of this pipeline, we have used `570.169`.
- Install Isaac Sim (4.5.0)
  - Follow the instructions for the version [4.5.0](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html)
- Install Isaac Lab by following the [installation guide](http://isaac-sim.github.io/IsaacLab/v2.1.1/source/setup/installation/binaries_installation.html).
  - When cloning the Isaaclab GitHub, make sure to change to the version v2.1.1 by:
    ```bash
    cd Isaaclab
    git checkout v2.1.1
    ```
  - After installing Miniconda, make sure you are not in an enviroment before you run:
    ```bash
    # Option 1: Default name for conda environment is 'env_isaaclab'
    ./isaaclab.sh --conda  # or "./isaaclab.sh -c"
    # Option 2: Custom name for conda environment
    ./isaaclab.sh --conda my_env  # or "./isaaclab.sh -c my_env"
    ```
- Install the Unitree RL Lab standalone environments.

  - Clone or copy this repository separately from the Isaac Lab installation (i.e. outside the `IsaacLab` directory):

    ```bash
    git clone https://github.com/unitreerobotics/unitree_rl_lab.git
    ```
  - Use a python interpreter that has Isaac Lab installed, install the library in editable mode using:

    ```bash
    conda activate env_isaaclab
    python -m pip install -e source/unitree_rl_lab
    ```
- Download unitree usd files

  - Download unitree usd files from [unitree_model](https://huggingface.co/datasets/unitreerobotics/unitree_model/tree/main), keeping folder structure
    ```bash
    git clone https://huggingface.co/datasets/unitreerobotics/unitree_model
    ```
  - If you don't have [LFS](https://docs.github.com/en/repositories/working-with-files/managing-large-files/installing-git-large-file-storage), by default it will clone only references to large files. After installing it, go to the `unitree_model` directory, install LFS and pull the large files.
    ```bash
    git lfs install
    git lfs pull
    ``` 
  - Config `UNITREE_MODEL_DIR` in `source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`.

    ```bash
    UNITREE_MODEL_DIR = "</home/user/projects/unitree_usd>"
    ```
- Verify that the environments are correctly installed by:

  - Listing the available tasks:

    ```bash
    python unitree_rl_lab/scripts/list_envs.py
    ```


### Training policy

- Running a task:

  ```bash
  python unitree_rl_lab/scripts/rsl_rl/train.py --headless --task Unitree-G1-29dof-Velocity
  ```

### Testing policy

- Inference with a trained agent:

  ```bash
  python unitree_rl_lab/scripts/rsl_rl/play.py --task Unitree-G1-29dof-Velocity
  ```

### Sim2Sim

After the model training is completed, we need to perform Sim2Sim on the trained strategy in MuJoCo to test the performance of the model.
Then deploy Sim2Real.

### Setup



### Sim2Sim

Install the [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco?tab=readme-ov-file#installation).

- Set the `robot` at `/simulate/config.yaml` to g1
- Set `domain_id` to 0
- Set `enable_elastic_band` to 1
- Set `use_joystck` to 1.

```bash
# start simulation
cd unitree_mujoco/simulate/build
sudo ./unitree_mujoco
```

```bash
cd unitree_rl_lab/deploy/robots/g1_29dof/build
sudo ./g1_ctrl --network lo
# 1. press [L2 + Up] to set the robot to stand up
# 2. Click the mujoco window, and then press 8 to make the robot feet touch the ground.
# 3. Press [R1 + X] to run the policy.
# 4. Click the mujoco window, and then press 9 to disable the elastic band.
```

### Sim2Real

You can use this program to control the robot directly, but make sure the on-board control program has been closed.

```bash
sudo ./g1_ctrl --network <eth0/enp2s0/etc> # the network interface name.
```

## Acknowledgements

This repository is built upon the support and contributions of the following open-source projects. Special thanks to:

- [IsaacLab](https://github.com/isaac-sim/IsaacLab): The foundation for training and running codes.
- [mujoco](https://github.com/google-deepmind/mujoco.git): Providing powerful simulation functionalities.
- [robot_lab](https://github.com/fan-ziqi/robot_lab): Referenced for project structure and parts of the implementation.
