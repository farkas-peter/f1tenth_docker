#!/bin/bash

# Simple script to verify Python 3.10+ is available in the container

echo "Testing Python version in the container..."
echo "=========================================="

# Check if container is running
CONTAINER_NAME="f1tenth_developer_container"
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Please start the container first using: ./start_client.sh"
    exit 1
fi

# Test Python version
echo "Running: docker exec ${CONTAINER_NAME} python3 --version"
PYTHON_VERSION=$(docker exec ${CONTAINER_NAME} python3 --version 2>&1)
echo "Result: $PYTHON_VERSION"

# Extract version number and check if it's >= 3.10
if echo "$PYTHON_VERSION" | grep -q "Python 3\.1[0-9]"; then
    echo "✓ SUCCESS: Python 3.10+ is installed and set as default"
    exit 0
else
    echo "✗ FAIL: Python version is not 3.10+"
    exit 1
fi
