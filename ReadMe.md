# Docker for developing F1TENTH Project

---

## Using the container on external laptop or PC
Changes in the `Dockerfile`:
- remove the cloning of `f1tenth_system` repository
- remove `--include-eol-distros` from the `rosdep update` command 

Changes in the `docker-compose.yml`:
- remove the `runtime: nvidia` line
- create an `F1TENTH_LOG` directory in your workspace (near the `f1tenth_docker` folder)

---

## SSH Connection
1. Connect to the WiFi network of the F1TENTH car
    - Name: `f1tenth`
    - Pwd: `f1tenth_2025`


2. SSH to the Jetson:
    - Command: `ssh dev@10.0.10.1`
    - Pwd: `dev`


3. SSH to the running container:
    - Command: `ssh f1tenth@10.0.10.1 -p 24`
    - Pwd: `f1tenth`

---

## General
The setup contains a ROS2 (/workspace) setup that contains the used repositories in built form.

A terminal window will open with "f1tenth" user in /workspace folder. 

The container uses host networking, means it has not its own IP and network layer, it uses the host machine IP address and network configuration.

ROS2 discovery server is set, all the ROS2 nodes in the ROS2 network have to use that.

SSH service of the container uses port 24.

The container builds the system and starts it automatically in case of computers without DISPLAY environment variable.

---

<span style="color:red; font-weight:bold">Warning: If the docker image is rebuilt or any changes are made in the docker-compose file, then the 
earlier used container will be deleted with all the modification within it.</span>

---

### Files:
- `Dockerfile`

    - On the Jetson, based on: [ultralytics/ultralytics:latest-jetson-jetpack5](https://hub.docker.com/layers/ultralytics/ultralytics/latest-jetson-jetpack5/images/sha256-5f3d98f3d08b7d72142f7f720c8d6080a0d3138abb25bb9250c6ad1d6ea05a0c?context=explore), which is built from [nvcr.io/nvidia/l4t-jetpack:r35.4.1](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/tags) for YOLO applications
    - On external PC/Laptop, based on: [nvidia/cuda:11.6.1-base-ubuntu20.04](https://hub.docker.com/layers/nvidia/cuda/11.6.1-base-ubuntu20.04/images/sha256-70e97597e8bbaba961143b380bae5b092d09bdbed622fdb3069a572df68e476d)
    - Contains basic applications.
    - Main working folders are created.

- `docker-compose.yml`
    - Contains a F1TENTH_developer_container service, that will create the container. 
    - If the host PC has nvidia gpu, it is recommended to use nvidia runtime in the docker-compose file. 
    - Additional files, folders and volumes of the host PC can be added to the container.
    - For example:
       - `./entry.sh:/entry.sh`
       - `source_path:target_path`

- `entry.sh`
   - Start ssh service.
   - Configure the discovery server of ROS2 and set the actual ip4 address
   - Based on the DISPLAY environment variable that is set in the docker-compose.yml, the container opens a terminator window or be in stand-by state(/bin/bash).
   - Build the system and start it in case of computer without DISPLAY environment variable.
   - Any additional commands can be added before the "terminator" command, if they are required to run before the opening of the terminal window. 

- `build_client.sh`
   - Collects the UID, GID and passing to the Dockerfile as calibration and builds the docker image. 
   - Calibrate CAN, gpio, wifi and ethernet interfaces if they are missing.

- `start_client.sh`
   - Start the container based on the docker compose file.
   - If the docker image is missing, it will build it and automatically start the container, but highly recommended to use the build_client.sh, that uses additional settings.
   - If the container's terminal window is closed, the container will stop.

---

## First time usage
0. Probably need to install/upgrade docker => use `utility/docker_install.sh` and `utility/nvidia_container_install.sh`
1. Build the image with the `./build_client.sh`
2. Set the necessary volumes in the docker-compose.yml
    - Set the nvidia runtime if possible (nvidia GPU is required)
3. Start vscode with pre-installed Docker, Dev Container, Remote Explorer extensions and open the repo folder with it. The vscode will detect the devcontainer.json and start the container based on the `docker-compose.yml`. After that the vscode will attach to the container install the extensions and development can be done

Recommended to wait until the extensions are installed before the vscode closed again.

---

## Recomended VS Code externsions
- Docker: ms-azuretools.vscode-docker
- Dev Container: ms-vscode-remote.remote-containers
- Remote Explorer: ms-vscode.remote-explorer

- CMake: twxs.cmake
- CMake Language support: josetr.cmake-language-support-vscode
- CMake tools: ms-vscode.cmake-tools
- ROS: ms-iot.vscode-ros
- TODO tree: gruntfuggly.todo-tree

---

## Link collection
Use Yolo on Jetson platforms:
- https://docs.ultralytics.com/guides/nvidia-jetson/

RealSense SDK install for Jetson:
- https://github.com/intelRealSense/librealsense/blob/master/doc/installation_jetson.md

F1Tenth's website:
- https://roboracer.ai/build

## RealSense SDK install for Jetson
Run this shellscript in utility folder to install RealSense SDK:
- rs_sdk_install.sh

After the installation, run the following to open the RealSense SDK:
- realsense-viewer


## Command collection
- Starting F1 stack: `ros2 launch f1tenth_stack bringup_launch.py`
- Intel RealSense starting camera: `ros2 launch realsense2_camera rs_launch.py`
- Starting Rviz2: `rviz2`
- ROS node topology: `ros2 run rqt_graph rqt_graph`
![plot](utility/rqt_graph_final.png)

- ...