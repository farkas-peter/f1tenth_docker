#! /bin/bash

#docker compose build
export HOST_GID=$(id -g)
export HOST_UID=$(id -u)

source .env

# # Set base image based on the architecture id  of the host machine 
export BASE_IMG="nvidia/cuda:12.2.0-base-ubuntu22.04"

if [ "$(dpkg --print-architecture)" == "amd64" ]; then
    BASE_IMG="nvidia/cuda:12.2.0-base-ubuntu22.04"
elif [ "$(dpkg --print-architecture)" == "arm64" ]; then
    BASE_IMG="ultralytics/ultralytics:latest-jetson-jetpack5"
else
    echo "Error: Unsupported architecture."
    exit 1
fi

docker build\
    --rm --network=host \
    --build-arg HOST_UID=$HOST_UID \
    --build-arg HOST_GID=$HOST_GID \
    --build-arg BASE_IMG=$BASE_IMG \
    --tag f1tenth_developer:$IM_VERSION \
    --file Dockerfile .
