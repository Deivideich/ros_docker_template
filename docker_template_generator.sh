#!/bin/bash

# Docker Template Generator
# Auto-generated script to recreate the Docker development environment
# This script is self-contained and portable

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="${1:-application_sim}"
DOCKER_REGISTRY="${2:-roborregos}"

echo "🚀 Generating Docker template for project: $PROJECT_NAME"
echo "📦 Using Docker registry: $DOCKER_REGISTRY"

# Create directory structure
create_directory_structure() {
    echo "📁 Creating directory structure..."
    mkdir -p "${PROJECT_NAME}/docker/compose"
    mkdir -p "${PROJECT_NAME}/docker/dockerfiles"
    mkdir -p "${PROJECT_NAME}/docker/env_files"
    mkdir -p "${PROJECT_NAME}/docker/scripts"
    echo "✅ Directory structure created"
}

# Start file creation functions


# Create scripts/application_sim.sh
create_scripts_application_sim_sh() {
    cat > "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh" << 'EOF_SCRIPTS_APPLICATION_SIM_SH'
#!/bin/bash

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE_PATH="$PROJECT_ROOT/compose/docker-compose.yml"

# Load environment variables from .env file
if [ -f "$PROJECT_ROOT/compose/.env" ]; then
    export $(grep -v '^#' "$PROJECT_ROOT/compose/.env" | xargs)
else
    echo "❌ Error: .env file not found in $PROJECT_ROOT/compose/"
    exit 1
fi

# Util
function parse_gpu_flag() {
    local use_gpu=false
    for arg in "$@"; do
        if [[ "$arg" == "--gpu" ]]; then
            use_gpu=true
        fi
    done
    echo "$use_gpu"
}

function parse_cuda_flag() {
    local use_cuda=false
    for arg in "$@"; do
        if [[ "$arg" == "--cuda" ]]; then
            use_cuda=true
        fi
    done
    echo "$use_cuda"
}

function parse_cpu_flag() {
    local use_cpu=false
    for arg in "$@"; do
        if [[ "$arg" == "--cpu" ]]; then
            use_cpu=true
        fi
    done
    echo "$use_cpu"
}

# Utility to determine CUDA support
function has_cuda_support() {
    local use_CUDA=false
    for arg in "$@"; do
        if [[ "$arg" == "--cuda" ]]; then
            use_CUDA=true
        fi
    done
    echo "$use_CUDA"
}

function get_base_tag() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    cuda_flag=$(has_cuda_support "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        echo "$CPU_BASE_IMAGE_TAG"
    elif [[ "$cuda_flag" == "true" ]]; then
        echo "$CUDA_BASE_IMAGE_TAG"
    else
        echo "$CPU_BASE_IMAGE_TAG"
    fi
}

function get_application_sim_service() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        echo "application_gpu"
    elif [[ "$(has_cuda_support "$@")" == "true" ]]; then
        echo "application_cuda"
    else
        echo "application_cpu"
    fi
}

function get_application_sim_container_name() {
    local gpu_flag
    gpu_flag=$(parse_gpu_flag "$@")
    if [[ "$gpu_flag" == "true" ]]; then
        echo "application_sim_gpu"
    elif [[ "$(has_cuda_support "$@")" == "true" ]]; then
        echo "application_sim_cuda"
    else
        echo "application_sim_cpu"
    fi
}

# Step-by-step helpers
function build_base_image() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    echo "🚧 Building base image: $base_tag"
    docker compose -f "$COMPOSE_FILE_PATH" build "$base_tag"
}

function build_application_sim_image() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    local service
    service=$(get_application_sim_service "$@")

    echo "🔧 Building application_sim image: $service (based on $base_tag)"
    export application_sim_BASE_IMAGE="$DOCKER_REGISTRY/$PROJECT_NAME:$base_tag"
    export application_sim_BASE_IMAGE_TAG="$base_tag"
    docker compose -f "$COMPOSE_FILE_PATH" build "$service"
}

function run_container() {
    local base_tag
    base_tag=$(get_base_tag "$@")
    local base_image="$DOCKER_REGISTRY/$PROJECT_NAME:$base_tag"
    local service
    service=$(get_application_sim_service "$@")
    local container_name
    container_name=$(get_application_sim_container_name "$@")

    echo "🚀 Starting container: $service"
    xhost +local:docker
    application_sim_BASE_IMAGE="$base_image" \
    application_sim_BASE_IMAGE_TAG="$base_tag" \
    docker compose -f "$COMPOSE_FILE_PATH" up -d "$service"
    until docker exec -it "$container_name" bash -c "ls /tmp/build_done" &>/dev/null; do
        echo "⏳ Waiting for build to complete..."
        sleep 2
    done
    echo "✅ Done"
}

function attach_shell() {
    local container_name
    container_name=$(get_application_sim_container_name "$@")

    echo "🧑‍💻 Attaching to $container_name shell..."
    docker exec -it "$container_name" bash
}

# Top-level operations
function deploy() {
    # Require at least one argument (e.g., --gpu, --cuda, or default)
    if [[ $# -lt 2 ]]; then
        echo "❌ Error: Please specify a target to deploy (e.g., --gpu, --cuda)"
        exit 1
    fi

    if [[ "$(parse_gpu_flag "$@")" == "true" ]]; then
        echo "🚀 Deploying with GPU support..."
    elif [[ "$(parse_cuda_flag "$@")" == "true" ]]; then
        echo "🚀 Deploying with CUDA support..."
    elif [[ "$(parse_cpu_flag "$@")" == "true" ]]; then
        echo "🚀 Deploying with CPU support..."
    else
        echo "❌ Error: No valid target specified. Please use --gpu, --cuda, or --cpu."
        exit 1
    fi

    build_base_image "$@"
    build_application_sim_image "$@"
    run_container "$@"
    attach_shell "$@"
}


function dev_mode() {
    run_container "$@"
    attach_shell "$@"
}

function stop_container() {
    # At least one argument is required after -stop
    if [[ $# -lt 2 ]]; then
        echo "❌ Error: Please specify a container to stop (e.g., --gpu, --cuda, or 'all')"
        exit 1
    fi

    # Check for "all"
    for arg in "$@"; do
        if [[ "$arg" == "all" ]]; then
            echo "🛑 Stopping all containers from docker-compose.yml..."
            docker compose -f "$COMPOSE_FILE_PATH"  stop
            return
        fi
    done

    # Otherwise stop the specific one
    local service
    service=$(get_application_sim_service "$@")

    echo "🛑 Stopping container: $service"
    docker compose -f "$COMPOSE_FILE_PATH"  stop "$service"
}

function remove_container() {
    # At least one argument is required after -remove
    if [[ $# -lt 2 ]]; then
        echo "❌ Error: Please specify a container to remove (e.g., --gpu, --cuda, or 'all')"
        exit 1
    fi

    # Check for "all"
    for arg in "$@"; do
        if [[ "$arg" == "all" ]]; then
            echo "🛑 Removing all containers from docker-compose.yml..."
            docker compose -f "$COMPOSE_FILE_PATH" down
            return
        fi
    done

    # Otherwise remove the specific service
    local service
    service=$(get_application_sim_service "$@")

    echo "🛑 Removing container: $service"
    docker compose -f "$COMPOSE_FILE_PATH" down "$service"
}



# function remove_all() {
#     echo "🧹 Removing all containers and resources..."
#     docker compose -f "$COMPOSE_FILE_PATH"  down
# }

function help_message() {
    echo "Usage: ./application_sim.sh [COMMAND] [--gpu]"
    echo
    echo "Commands:"
    echo "  -build-base [--gpu]      Build only the base image"
    echo "  -build-development-image [--gpu]      Build only the APPLICATION image"
    echo "  -run [--gpu]             Start only the container"
    echo "  -dev-mode [--gpu]        Run + attach to shell"
    echo "  -deploy [--gpu|--cuda]       Build everything, run, and attach"
    echo "  -remove                  Remove all containers"
    echo "  -help                    Show this help message"
    echo
    echo "Examples:"
    echo "  ./application_sim.sh -deploy"
    echo "  ./application_sim.sh -dev-mode --gpu"
    echo "  ./application_sim.sh -build-base"
}

# Main dispatcher
case "$1" in
    -build-base)
        build_base_image "$@"
        ;;
    -build-development-image)
        build_application_sim_image "$@"
        ;;
    -run)
        run_container "$@"
        ;;
    -dev-mode)
        dev_mode "$@"
        ;;
    -deploy)
        deploy "$@"
        ;;
    -remove)
        remove_container "$@"
        ;;
    -help|--help)
        help_message
        ;;
    -stop)
        stop_container "$@"
        ;;
    *)
        echo "Unknown command: $1"
        help_message
        exit 1
        ;;
esac
EOF_SCRIPTS_APPLICATION_SIM_SH
    
    # Replace template variables with actual values
    sed -i "s/application_sim/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    sed -i "s/test_project/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    sed -i "s/APPLICATION_SIM/${PROJECT_NAME^^}/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    sed -i "s/TEST_PROJECT/${PROJECT_NAME^^}/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    # Replace function names and variables that contain the project name
    sed -i "s/get_application_sim_service/get_${PROJECT_NAME}_service/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    sed -i "s/build_application_sim_image/build_${PROJECT_NAME}_image/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    sed -i "s/application_sim_BASE_IMAGE/${PROJECT_NAME}_BASE_IMAGE/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    sed -i "s/application_sim_BASE_IMAGE_TAG/${PROJECT_NAME}_BASE_IMAGE_TAG/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    # Replace help text and comments
    sed -i "s/vsss_sim\.sh/${PROJECT_NAME}.sh/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    sed -i "s/VSSS/${PROJECT_NAME^^}/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
    # Replace workspace paths for all script files
    sed -i "s/application_ws/${PROJECT_NAME}_ws/g" "${PROJECT_NAME}/docker/scripts/${PROJECT_NAME}.sh"
}

# Create scripts/entrypoint.sh
create_scripts_entrypoint_sh() {
    cat > "${PROJECT_NAME}/docker/scripts/entrypoint.sh" << 'EOF_SCRIPTS_ENTRYPOINT_SH'
#!/bin/bash
set -e

# Source ROS
source /opt/ros/humble/setup.bash

#rosdep
rosdep update
rosdep install --from-paths /ros/application_ws/src --ignore-src -r -y

# Optional: source overlay if needed
if [ -f "/ros/application_ws/install/setup.bash" ]; then
  source /ros/application_ws/install/setup.bash
fi

# Build
cd /ros/application_ws
colcon build --symlink-install
touch /tmp/build_done

# Keep container alive interactively if no other command was passed
if [ "$#" -eq 0 ]; then
  exec bash
else
  exec "$@"
fi
EOF_SCRIPTS_ENTRYPOINT_SH
    
    # Replace template variables with actual values
    sed -i "s/application_sim/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    sed -i "s/test_project/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    sed -i "s/APPLICATION_SIM/${PROJECT_NAME^^}/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    sed -i "s/TEST_PROJECT/${PROJECT_NAME^^}/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    # Replace function names and variables that contain the project name
    sed -i "s/get_application_sim_service/get_${PROJECT_NAME}_service/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    sed -i "s/build_application_sim_image/build_${PROJECT_NAME}_image/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    sed -i "s/application_sim_BASE_IMAGE/${PROJECT_NAME}_BASE_IMAGE/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    sed -i "s/application_sim_BASE_IMAGE_TAG/${PROJECT_NAME}_BASE_IMAGE_TAG/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    # Replace help text and comments
    sed -i "s/vsss_sim\.sh/${PROJECT_NAME}.sh/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    sed -i "s/VSSS/${PROJECT_NAME^^}/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
    # Replace workspace paths for all script files
    sed -i "s/application_ws/${PROJECT_NAME}_ws/g" "${PROJECT_NAME}/docker/scripts/entrypoint.sh"
}

# Create compose/docker-compose.yml
create_compose_docker_compose_yml() {
    cat > "${PROJECT_NAME}/docker/compose/docker-compose.yml" << 'EOF_COMPOSE_DOCKER_COMPOSE_YML'
services:
  cpu_base:
    build:
      context: ${BUILD_CONTEXT}
      dockerfile: ${ROS_DOCKERFILE}
      args:
        BASE_IMAGE: ${CPU_BASE_IMAGE}
    image: ${DOCKER_REGISTRY}/${PROJECT_NAME}:cpu_base

  cuda_base:
    build:
      context: ${BUILD_CONTEXT}
      dockerfile: ${ROS_DOCKERFILE}
      args:
        BASE_IMAGE: ${CUDA_BASE_IMAGE}
    image: ${DOCKER_REGISTRY}/${PROJECT_NAME}:cuda_base

  application_cuda:
    build:
      context: ${BUILD_CONTEXT}
      dockerfile: ${APPLICATION_SIM_DOCKERFILE}
      args:
        BASE_IMAGE: ${BUILT_CUDA_BASE_IMAGE}
    image: ${DOCKER_REGISTRY}/${PROJECT_NAME}_cuda
    container_name: ${PROJECT_NAME}_cuda
    runtime: ${NVIDIA_RUNTIME}
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES}
      - LIBGL_ALWAYS_INDIRECT=${LIBGL_ALWAYS_INDIRECT}
    volumes:
      - ${X11_SOCKET_PATH}:${X11_SOCKET_PATH}
      - ${WORKSPACE_PATH}:/ros/application_ws/src/
    devices:
      - /dev/dri:/dev/dri
    network_mode: ${NETWORK_MODE}
    privileged: ${PRIVILEGED}
    stdin_open: ${STDIN_OPEN}
    tty: ${TTY}
    command: []
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: ${GPU_COUNT}
              capabilities: [gpu]

  application_gpu:
    build:
      context: ${BUILD_CONTEXT}
      dockerfile: ${APPLICATION_SIM_DOCKERFILE}
      args:
        BASE_IMAGE: ${DOCKER_REGISTRY}/${PROJECT_NAME}:cpu_base
    image: ${DOCKER_REGISTRY}/${PROJECT_NAME}_gpu
    container_name: ${PROJECT_NAME}_gpu
    runtime: ${NVIDIA_RUNTIME}
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES}
      - LIBGL_ALWAYS_INDIRECT=${LIBGL_ALWAYS_INDIRECT}
    volumes:
      - ${X11_SOCKET_PATH}:${X11_SOCKET_PATH}
      - ${WORKSPACE_PATH}:/ros/application_ws/src/
    devices:
      - /dev/dri:/dev/dri
    network_mode: ${NETWORK_MODE}
    privileged: ${PRIVILEGED}
    stdin_open: ${STDIN_OPEN}
    tty: ${TTY}
    command: []

  application_cpu:
    build:
      context: ${BUILD_CONTEXT}
      dockerfile: ${APPLICATION_SIM_DOCKERFILE}
      args:
        BASE_IMAGE: ${BUILT_CPU_BASE_IMAGE}
    image: ${DOCKER_REGISTRY}/${PROJECT_NAME}_cpu
    container_name: ${PROJECT_NAME}_cpu
    network_mode: ${NETWORK_MODE}
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - ${X11_SOCKET_PATH}:${X11_SOCKET_PATH}
      - ${WORKSPACE_PATH}:/ros/application_ws/src/
    stdin_open: ${STDIN_OPEN}
    tty: ${TTY}
    privileged: ${PRIVILEGED}
    command: []
EOF_COMPOSE_DOCKER_COMPOSE_YML
    
    # Replace template variables with actual values
    sed -i "s/application_sim/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/compose/docker-compose.yml"
    sed -i "s/test_project/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/compose/docker-compose.yml"
    sed -i "s/APPLICATION_SIM_DOCKERFILE/${PROJECT_NAME^^}_DOCKERFILE/g" "${PROJECT_NAME}/docker/compose/docker-compose.yml"
    sed -i "s/APPLICATION_TEST_PROJECT_DOCKERFILE/${PROJECT_NAME^^}_DOCKERFILE/g" "${PROJECT_NAME}/docker/compose/docker-compose.yml"
    # Replace workspace paths
    sed -i "s/application_ws/${PROJECT_NAME}_ws/g" "${PROJECT_NAME}/docker/compose/docker-compose.yml"
}

# Create compose/.env
create_compose__env() {
    cat > "${PROJECT_NAME}/docker/compose/.env" << 'EOF_COMPOSE__ENV'
# Base images
CPU_BASE_IMAGE=ubuntu:22.04
CUDA_BASE_IMAGE=nvidia/cuda:11.8.0-runtime-ubuntu22.04
application_sim_BASE_IMAGE=ubuntu:22.04
application_sim_BASE_IMAGE_TAG=latest
BASE_IMAGE=roborregos/application_sim:gpu_base

# Docker registry and image names
DOCKER_REGISTRY=roborregos
PROJECT_NAME=application_sim

# Resulting base image names
BUILT_CPU_BASE_IMAGE=roborregos/application_sim:cpu_base
BUILT_CUDA_BASE_IMAGE=roborregos/application_sim:cuda_base
CPU_BASE_IMAGE_TAG=cpu_base
CUDA_BASE_IMAGE_TAG=cuda_base

# Display settings
DISPLAY=:0

# Volume paths
WORKSPACE_PATH=../../
X11_SOCKET_PATH=/tmp/.X11-unix

# Container settings
NETWORK_MODE=host
PRIVILEGED=true
STDIN_OPEN=true
TTY=true

# GPU settings
NVIDIA_RUNTIME=nvidia
NVIDIA_VISIBLE_DEVICES=all
LIBGL_ALWAYS_INDIRECT=0
GPU_COUNT=1

# Build context
BUILD_CONTEXT=../
ROS_DOCKERFILE=dockerfiles/ros.dockerfile
APPLICATION_DOCKERFILE=dockerfiles/application.dockerfile
APPLICATION_SIM_DOCKERFILE=dockerfiles/application_sim.dockerfile
EOF_COMPOSE__ENV
    
    # Replace template variables with actual values
    sed -i "s/application_sim/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/compose/.env"
    sed -i "s/test_project/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/compose/.env"
    sed -i "s/roborregos/$DOCKER_REGISTRY/g" "${PROJECT_NAME}/docker/compose/.env"
    # Replace dockerfile environment variables
    sed -i "s/APPLICATION_SIM_DOCKERFILE/${PROJECT_NAME^^}_DOCKERFILE/g" "${PROJECT_NAME}/docker/compose/.env"
    sed -i "s/APPLICATION_DOCKERFILE/APPLICATION_DOCKERFILE/g" "${PROJECT_NAME}/docker/compose/.env"
}

# Create dockerfiles/application_sim.dockerfile
create_dockerfiles_application_sim_dockerfile() {
    cat > "${PROJECT_NAME}/docker/dockerfiles/${PROJECT_NAME}.dockerfile" << 'EOF_DOCKERFILES_APPLICATION_SIM_DOCKERFILE'
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install Nav2 and common dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
# TODO ----Install ros2 dependencies ----
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Optional dev tools
RUN apt-get update && apt-get install -y \
    terminator nano net-tools iputils-ping

# Setup ROS workspace directory and permissions
RUN mkdir -p /ros/application_ws/src && \
    chown -R ros:ros /ros

# Clean any existing rosdep data and initialize rosdep (run as root)
RUN rm -rf /etc/ros/rosdep/sources.list.d/* /var/lib/rosdep/* && \
    rosdep init && \
    rosdep fix-permissions && \
    rosdep update
    
# Install additional ROS packages if needed
COPY ../scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Switch to non-root user
USER ros
WORKDIR /ros/application_ws

# Update rosdep db and install dependencies (run as ros user)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Source ROS 2 setup on login
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
EOF_DOCKERFILES_APPLICATION_SIM_DOCKERFILE
    
    # Replace template variables with actual values  
    sed -i "s/application_sim/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/dockerfiles/${PROJECT_NAME}.dockerfile"
    sed -i "s/test_project/$PROJECT_NAME/g" "${PROJECT_NAME}/docker/dockerfiles/${PROJECT_NAME}.dockerfile"
    # Replace workspace paths in dockerfiles
    sed -i "s/application_ws/${PROJECT_NAME}_ws/g" "${PROJECT_NAME}/docker/dockerfiles/${PROJECT_NAME}.dockerfile"
}

# Create dockerfiles/ros.dockerfile
create_dockerfiles_ros_dockerfile() {
    cat > "${PROJECT_NAME}/docker/dockerfiles/ros.dockerfile" << 'EOF_DOCKERFILES_ROS_DOCKERFILE'
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Cuda images: BASE_IMAGE: nvidia/cuda:11.8.0-runtime-ubuntu22.04
# CPU images: BASE_IMAGE: ubuntu:22.04
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=humble \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    QT_X11_NO_MITSHM=1

# Install language
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*



# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    curl \
    software-properties-common

# Install ROS
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt install -y ros-humble-desktop 

# Install ROS utilities
RUN apt-get update && apt-get install -y ros-dev-tools

# Setup workspace directory
RUN mkdir -p /workspace && chown -R $USERNAME:$USERNAME /workspace
WORKDIR /workspace

# Set up autocompletion for user
RUN apt-get update && apt-get install -y git-core bash-completion \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
    && rm -rf /var/lib/apt/lists/* 

# Source ROS workspace
# RUN echo "source /workspace/install/setup.bash" >> /home/$USERNAME/.bashrc

# Install general utilities
RUN apt-get update -qq && apt-get install -y  build-essential \
    ffmpeg libsm6 libxext6 autoconf libtool mesa-utils \
    terminator nano git wget iputils-ping \
    libcanberra-gtk-module libcanberra-gtk3-module \
    python3-pip
EOF_DOCKERFILES_ROS_DOCKERFILE
}

# Main execution function
main() {
    echo "🎯 Starting Docker template generation..."
    
    # Create base directory structure
    create_directory_structure
    
    # Create all files
    create_scripts_application_sim_sh
    create_scripts_entrypoint_sh
    create_compose_docker_compose_yml
    create_compose__env
    create_dockerfiles_application_sim_dockerfile
    create_dockerfiles_ros_dockerfile
    
    # Make scripts executable
    chmod +x "${PROJECT_NAME}/docker/scripts/"*.sh
    
    echo "✅ Docker template generated successfully!"
    echo ""
    echo "📋 Next steps:"
    echo "1. Navigate to the project directory: cd ${PROJECT_NAME}"
    echo "2. Navigate to the docker/compose directory: cd docker/compose"
    echo "3. Review and customize the .env file if needed"
    echo "4. Build and run your application:"
    echo "   ./docker/scripts/${PROJECT_NAME}.sh -deploy --gpu"
    echo ""
    echo "🔧 Available commands:"
    echo "   ./docker/scripts/${PROJECT_NAME}.sh -help"
}

# Script usage
show_usage() {
    echo "Usage: $0 [PROJECT_NAME] [DOCKER_REGISTRY]"
    echo ""
    echo "Arguments:"
    echo "  PROJECT_NAME     Name of your project (default: application_sim)"
    echo "  DOCKER_REGISTRY  Docker registry to use (default: roborregos)"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Use defaults"
    echo "  $0 my_robot_project                   # Custom project name"
    echo "  $0 my_robot_project my_registry       # Custom project and registry"
}

# Parse command line arguments
case "$1" in
    -h|--help|help)
        show_usage
        exit 0
        ;;
    *)
        main
        ;;
esac
