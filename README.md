# OPEN: Openstreetmap-enhanced oPen-air sEmantic Navigation

OPEN is a system designed for efficient last-mile delivery using autonomous robots. It leverages OpenStreetMap (OSM) for scalable map representation, Large Language Models (LLMs) for understanding delivery instructions, and Vision-Language Models (VLMs) for localization, map updates, and house number recognition.

## Installation

The current development environment is Ubuntu 22.04, ROS2 Humble, and Gazebo Classic 11.

### Using Docker

This project includes a basic Dockerfile and can be used with [Dev Container](https://containers.dev/) for simulation testing and development.

1. Install Docker and NVIDIA Container Toolkit and configure them on the host machine:

    ```bash
    # Docker installation
    https://docs.docker.com/engine/install/ubuntu/
    wget http://fishros.com/install -O fishros (optional)
    
    # NVIDIA Container Toolkit installation
    https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

    # nvidia-docker installation
    sudo apt install nvidia-docker2
    
    # Configure Docker container to use GPU
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ```

2. Download and install the image:

    ```sh
    # Download from Baidu Cloud
    Link: https://pan.baidu.com/s/1xSDHBTYh8PTt4HNBLy7FWA?pwd=emgo 
    Password: emgo 
    
    # Load the image
    docker load -i open.tar
    ```
    
3. Clone the repository:
    ```sh
    git clone https://github.com/EI-Nav/light-map-navigation.git --depth=1
    git submodule init
    git submodule update

    cd src/classic_localization/FAST_LIO/
    git submodule init
    git submodule update
    ```

4. Install the `ms-vscode-remote.remote-containers` extension in VSCode on the host machine.

5. Open this project in VSCode, press `Ctrl+Shift+P`, type and select `Dev Containers: Rebuild and Reopen in Container`.

6. Install dependencies and compile:
    ```sh
    # Set up your proxy
    export https_proxy=http://127.0.0.1:7890 http_proxy=http://127.0.0.1:7890 all_proxy=socks5://127.0.0.1:7890

    # Install dependencies
    ./build_dependencies.sh

    # Compile
    ./build_project.sh
    ```

### Source Installation

1. Clone the repository:

    ```sh
    git clone https://github.com/EI-Nav/light-map-navigation.git --depth=1
    git submodule init
    git submodule update

    cd src/classic_localization/FAST_LIO/
    git submodule init
    git submodule update
    ```

2. Install [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2):

    ```sh
    sudo apt install cmake
    ```

    ```sh
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```

3. Install dependencies:

    ```sh
    cd light-map-navigation

    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

    pip3 install -r requirements.txt
    ```

4. Compile:

    ```sh
    ./build.sh
    ```

## Configuration

To use this repository, you'll need to configure the following credentials:

### VLM API

```yaml
# /workspace/light-map-navigation/src/building_entrance_recognition/config/api_config.yaml
llm_api:
  key: "your_key"
  base_url: "your_base_url"
  model_name: "your_model_name" 
```

## Run

```sh
# Allow the container to access the host's X server
xhost +
```

### Simulation Mode Example

```sh
# Run the robot's navigation system and simulation environment
ros2 launch classic_nav_bringup bringup_sim.launch.py \
world:=MEDIUM_OSM \
mode:=nav \
lio:=fastlio \
localization:=icp \
lio_rviz:=False \
nav_rviz:=True \
use_sim_time:=True

# Run delivery-related nodes
ros2 launch delivery_bringup delivery_system_sim.launch.py

# Run client to send delivery requests
ros2 run delivery_executor delivery_executor_action_client
```

### Grounded-SAM-2 Server
If you want to use Grounded-SAM-2 for open-vocabulary instance segmentation, you can run the following commands:
```sh
# Set up your proxy
export https_proxy=http://127.0.0.1:7890 http_proxy=http://127.0.0.1:7890 all_proxy=socks5://127.0.0.1:7890

# Download SAM2 checkpoints
cd src/grounded_sam2/grounded_sam2/Grounded-SAM-2/checkpoints
bash download_ckpts.sh

# Download GroundingDINO checkpoints
cd src/grounded_sam2/grounded_sam2/Grounded-SAM-2/gdino_checkpoints
bash download_ckpts.sh

# Run Grounded-SAM-2 server
cd /workspaces/light-map-navigation
./start_gsam2_server.sh

# Run Grounded-SAM-2 client --- refer to (src/grounded_sam2/grounded_sam2/grounded_sam2_client.py) for your own usage
source install/setup.bash
ros2 run grounded_sam2 grounded_sam2_client
```

## Robot Model

Four-wheeled vehicle using a differential drive model.

## Known Issues

- In some cases, the control system struggles to reach waypoints (zigzagging behavior).
- In ICP localization mode, there are coordinate system issues, with the vehicle appearing below the map.

## TODO

- Delete old delivery code, models, and tests.
- Improve documentation.
- Multimodal localization module based on factor graphs.

## Acknowledgments

Referenced repository:

https://github.com/LihanChen2004/pb_rm_simulation?tab=readme-ov-file