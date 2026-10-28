# 1. Install Docker, Compose and Buildx from Ubuntu's repo
sudo apt install -y docker.io docker-compose-v2 docker-buildx

# 2. Start Docker now and on every boot
sudo systemctl enable --now docker

# 3. Let your user run docker without sudo
sudo usermod -aG docker $USER
# Then log out and back in (PyCharm needs this too)

# 4. Test
docker run hello-world
docker compose version
docker buildx version
