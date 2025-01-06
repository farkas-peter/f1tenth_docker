#!/bin/bash
set -e

# This script automates the following instructions:
# https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

echo -e "\n\n\n Installing docker dependencies ..."
    
sudo apt-get update
sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

echo -e "\n\n\n Installing docker ..."
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin

## Check if user is not added to 'docker' group
if [ -z $"groups | grep -e 'docker'" ]; then
    echo -e "\n\n\n Adding docker to the user and group"
    sudo groupadd -f docker
    sudo usermod -aG docker $USER
    newgrp docker
fi


echo -e "\n\n\n Docker installed"

