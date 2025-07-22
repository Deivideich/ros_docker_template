# Docker ROS Template System

A portable and flexible Docker template system for ROS 2 projects that can be easily customized for any project name and Docker registry.

## 🚀 Quick Start

1. **Generate a new project:**

   ```bash
   ./docker_template_generator.sh my_project my_registry
   cd my_project
   ```

2. **Run your project:**

   ```bash
   ./docker/scripts/my_project.sh -deploy --gpu
   ```

## ✨ Features

- 🚀 **One-command project generation** - Create complete Docker environments instantly
- 🔧 **Automatic variable replacement** - Project names, registries, and workspace paths
- 🐳 **Multi-target support** - CPU, GPU, and CUDA configurations
- 📦 **Environment-driven configuration** - Customizable via `.env` files
- 🔄 **Portable and self-contained** - Single script contains entire template
- 🛠️ **ROS 2 ready** - Pre-configured for ROS 2 Humble development

## 📋 Project Structure

```text
docker/
├── compose/
│   ├── docker-compose.yml    # Multi-service Docker Compose
│   └── .env                  # Environment variables
├── dockerfiles/
│   ├── ros.dockerfile        # Base ROS 2 image
│   └── application_sim.dockerfile  # Application-specific image
└── scripts/
    ├── application_sim.sh    # Main management script
    └── entrypoint.sh         # Container entrypoint
```

## 🔧 Usage

### Generate New Projects

```bash
# Basic usage (defaults: application_sim, roborregos)
./docker_template_generator.sh

# Custom project name
./docker_template_generator.sh my_robot_project

# Custom project and registry
./docker_template_generator.sh my_robot_project my_registry
```

### Project Management Commands

```bash
# Build and deploy with GPU support
./docker/scripts/[PROJECT_NAME].sh -deploy --gpu

# Development mode (run + attach shell)
./docker/scripts/[PROJECT_NAME].sh -dev-mode --cpu

# Build only base images
./docker/scripts/[PROJECT_NAME].sh -build-base

# Build only application image
./docker/scripts/[PROJECT_NAME].sh -build-development-image --gpu

# Stop containers
./docker/scripts/[PROJECT_NAME].sh -stop --gpu
./docker/scripts/[PROJECT_NAME].sh -stop all

# Remove containers
./docker/scripts/[PROJECT_NAME].sh -remove --gpu
./docker/scripts/[PROJECT_NAME].sh -remove all

# Show help
./docker/scripts/[PROJECT_NAME].sh -help
```

### Available Flags

- `--gpu` - Use GPU-enabled containers with NVIDIA runtime
- `--cuda` - Use CUDA-enabled containers with device allocation
- `--cpu` - Use CPU-only containers (default)

## ⚙️ Configuration

The generated `.env` file allows customization of:

```bash
# Base images
CPU_BASE_IMAGE=ubuntu:22.04
CUDA_BASE_IMAGE=nvidia/cuda:11.8.0-runtime-ubuntu22.04

# Docker registry and project
DOCKER_REGISTRY=your_registry
PROJECT_NAME=your_project

# Display and volumes
DISPLAY=:0
WORKSPACE_PATH=../../
X11_SOCKET_PATH=/tmp/.X11-unix

# GPU settings
NVIDIA_RUNTIME=nvidia
NVIDIA_VISIBLE_DEVICES=all
GPU_COUNT=1
```

## 🛠️ Development

### Modifying the Template

1. Edit files in the `docker/` folder
2. Regenerate the template generator:

   ```bash
   ./make_docker_generator_file.sh
   ```

3. Test with a new project:

   ```bash
   ./docker_template_generator.sh test_project test_registry
   ```

### Template Structure

The template system automatically replaces:

- **Project names**: `application_sim` → `your_project`
- **Registry names**: `roborregos` → `your_registry`
- **Function names**: `get_application_sim_service` → `get_your_project_service`
- **Variable names**: `application_sim_BASE_IMAGE` → `your_project_BASE_IMAGE`
- **Workspace paths**: `application_ws` → `your_project_ws`
- **Service names**: `application_gpu` → `your_project_gpu`

## 🐳 Docker Services

The template generates three main services:

- **`{project}_cpu`** - CPU-only container for basic development
- **`{project}_gpu`** - GPU-enabled container with NVIDIA runtime
- **`{project}_cuda`** - CUDA-enabled container with device allocation

## 📦 Requirements

- Docker Engine with Compose V2
- NVIDIA Docker runtime (for GPU support)
- X11 forwarding support (for GUI applications)

## 🔍 Troubleshooting

### Common Issues

1. **Permission denied on scripts**:

   ```bash
   chmod +x docker/scripts/*.sh
   ```

2. **X11 forwarding not working**:

   ```bash
   xhost +local:docker
   ```

3. **NVIDIA runtime not found**:

   ```bash
   # Install nvidia-docker2
   sudo apt-get update
   sudo apt-get install nvidia-docker2
   sudo systemctl restart docker
   ```

### Debug Mode

Run with verbose output:

```bash
bash -x ./docker/scripts/[PROJECT_NAME].sh -deploy --gpu
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

1. Fork the repository
2. Make your changes in the `docker/` folder
3. Regenerate the template: `./make_docker_generator_file.sh`
4. Test your changes
5. Submit a pull request

## 📚 Examples

### Creating a Navigation Robot Project

```bash
./docker_template_generator.sh nav_robot my_company
cd nav_robot
./docker/scripts/nav_robot.sh -deploy --gpu
```

### Creating a Computer Vision Project

```bash
./docker_template_generator.sh vision_ai nvidia
cd vision_ai
./docker/scripts/vision_ai.sh -dev-mode --cuda
```
