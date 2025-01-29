#!/usr/bin/env bash

# Enable debug output
set -x

ROOT="$(dirname "$(readlink -f "$0")")"
INSTALL_PREFIX="/usr/local/bin"
VENV_PATH="$ROOT"/.venv
CACHE_DIR="$ROOT/.cache"
CACHE_FILE="$CACHE_DIR/apt_cache.txt"

mkdir -p "$CACHE_DIR"

check_cache() {
    local package=$1
    if [ -f "$CACHE_FILE" ]; then
        grep -q "^$package$" "$CACHE_FILE"
        return $?
    fi
    return 1
}

mark_cached() {
    local package=$1
    echo "$package" >> "$CACHE_FILE"
}

install_package() {
    local package=$1
    if ! check_cache "$package"; then
        sudo apt-get install -y "$package"
        mark_cached "$package"
    else
        echo "Package $package already installed (cached)"
    fi
}

# Update package lists only if haven't been updated in last hour
if [ ! -f "$CACHE_DIR/last_update" ] || [ $(( $(date +%s) - $(stat -c %Y "$CACHE_DIR/last_update") )) -gt 3600 ]; then
    sudo apt-get update
    touch "$CACHE_DIR/last_update"
fi

# Install required packages with caching
install_package "software-properties-common"

if ! grep -q "^deb.*deadsnakes/ppa" /etc/apt/sources.list /etc/apt/sources.list.d/*; then
    sudo add-apt-repository -y ppa:deadsnakes/ppa
    sudo apt-get update
    touch "$CACHE_DIR/last_update"
fi

install_package "python3.10"
install_package "python3.10-dev"
install_package "python3.10-venv"

# Virtual environment setup
if [ ! -d "$VENV_PATH" ]; then
    python3.10 -m venv "$VENV_PATH"
    "$VENV_PATH"/bin/pip install -r "$ROOT"/build-kit/requirements.txt
else
    echo "Virtual environment already exists"
fi

source "$VENV_PATH"/bin/activate