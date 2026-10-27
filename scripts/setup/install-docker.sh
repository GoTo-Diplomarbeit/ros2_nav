# 1. Install requirements
sudo apt install -y uidmap curl iptables

# 2. Load the nf_tables kernel module (now and on every boot)
sudo modprobe nf_tables
echo nf_tables | sudo tee /etc/modules-load.d/nf_tables.conf

# 3. Allow rootlesskit to use user namespaces (Ubuntu 24.04 AppArmor fix)
cat <<EOT | sudo tee /etc/apparmor.d/home.$USER.bin.rootlesskit
abi <abi/4.0>,
include <tunables/global>

/home/$USER/bin/rootlesskit flags=(unconfined) {
  userns,
}
EOT
sudo systemctl daemon-reload
sudo systemctl restart apparmor.service

# 4. Download the rootless Docker binaries (the setup step may error here, that's fine)
curl -fsSL https://get.docker.com/rootless | sh

# 5. Make ~/bin available now and permanently
export PATH=$HOME/bin:$PATH
echo 'export PATH=$HOME/bin:$PATH' >> ~/.bashrc
echo 'export DOCKER_HOST=unix:///run/user/$(id -u)/docker.sock' >> ~/.bashrc

# 6. Run the setup tool (creates and starts the user service)
dockerd-rootless-setuptool.sh install

# 7. Keep Docker running after logout and across reboots
sudo loginctl enable-linger $USER

# 8. Install the Compose and Buildx plugins (not included in the rootless installer)
mkdir -p ~/.docker/cli-plugins
curl -SL https://github.com/docker/compose/releases/latest/download/docker-compose-linux-x86_64 -o ~/.docker/cli-plugins/docker-compose
URL=$(curl -s https://api.github.com/repos/docker/buildx/releases/latest | grep browser_download_url | grep linux-amd64 | grep -v json | cut -d '"' -f 4)
curl -SL "$URL" -o ~/.docker/cli-plugins/docker-buildx
chmod +x ~/.docker/cli-plugins/docker-compose ~/.docker/cli-plugins/docker-buildx

# 9. PyCharm fixes (GUI apps don't read ~/.bashrc)
sudo ln -s $HOME/bin/docker /usr/local/bin/docker
echo 'export DOCKER_HOST=unix:///run/user/$(id -u)/docker.sock' >> ~/.profile
# Then log out and back in

# 10. Reload shell config and test
source ~/.bashrc
docker run hello-world
docker compose version
docker buildx version
