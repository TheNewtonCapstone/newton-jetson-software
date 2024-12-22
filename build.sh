#!/usr/bin/env bash
ROOT="$(dirname "$(readlink -f "$0")")"
PYTHONPATH="$PYTHONPATH:$ROOT" python3 -m build-kit.scripts.cli"$@"