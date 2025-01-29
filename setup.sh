# ROOT="$(dirname "$(readlink -f "$0")")"
ROOT=$(pwd)
echo $ROOT
INSTALL_PREFIX="/usr/local/bin"

# Make newton executable and create symlink
chmod +x "$ROOT/newton"
sudo ln -sf "$ROOT/newton" "$INSTALL_PREFIX/newton"
