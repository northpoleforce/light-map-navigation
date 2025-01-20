#!/bin/bash

# 定义颜色输出
GREEN='\033[0;32m'
NC='\033[0m'

# 定义conda环境变量
CONDA_ENV="dl_env"
CONDA_PATH="/opt/conda/bin/conda"
CONDA_PIP="pip"

# 定义辅助函数
log_step() {
    echo -e "\n${GREEN}=== $1 ===${NC}"
}

install_in_conda() {
    echo "Installing $1..."
    $CONDA_PATH run -n $CONDA_ENV $CONDA_PIP install $2
}

# 系统依赖安装
log_step "Starting Dependency Installation"
apt update -y
apt install -y python3-pip
pip3 install -r requirements.txt

# ROS依赖安装
log_step "Installing ROS Dependencies"
DEBIAN_FRONTEND=noninteractive apt install -y
rosdep install --from-paths src --ignore-src -r -y

# Conda环境配置
log_step "Configuring Python Environment"
echo "Installing libstdcxx-ng..."
$CONDA_PATH install -y -c conda-forge -n $CONDA_ENV libstdcxx-ng

# Python包安装
log_step "Installing Python Packages"
install_in_conda "Flask" "flask"
install_in_conda "grounded_sam2" "-e src/grounded_sam2/grounded_sam2/grounded_sam2"
install_in_conda "grounding_dino" "--no-build-isolation -e src/grounded_sam2/grounded_sam2/grounded_sam2/grounding_dino"