#!/usr/bin/env bash

# Enable debug output
set -x

ROOT="$(dirname "$(readlink -f "$0")")"
INSTALL_PREFIX="/usr/local/bin"
VENV_PATH="$ROOT"/.venv
CACHE_DIR="$ROOT/.cache"
CACHE_FILE="$CACHE_DIR/apt_cache.txt"

mkdir -p "$CACHE_DIR"

# Virtual environment setup
if [ -d "$VENV_PATH" ]; then
    rm -rf "$VENV_PATH/"
else
    echo "Virtual environment already exists"
fi

python3.10 -m venv "$VENV_PATH"
source "$VENV_PATH"/bin/activate
pip install --upgrade pip
pip install -r docker/requirements.txt
