# 1. Install the toolkit (lets Docker pass the GPU + driver libs into containers)
sudo pacman -S --needed nvidia-container-toolkit

# 2. Register the nvidia runtime with Docker and restart it
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 3. Test (should print the same GPU table as on the host)
docker run --rm --gpus all ubuntu nvidia-smi
