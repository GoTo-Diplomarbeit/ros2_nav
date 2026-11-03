# 1. Add NVIDIA's apt repo (nvidia-container-toolkit isn't in Ubuntu's repo)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. Install the toolkit (lets Docker pass the GPU + driver libs into containers)
sudo apt update
sudo apt install -y nvidia-container-toolkit

# 3. Register the nvidia runtime with Docker and restart it
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 4. Test (should print the same GPU table as on the host)
docker run --rm --gpus all ubuntu nvidia-smi
