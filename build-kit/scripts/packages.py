#!/usr/bin/env python3
import os

_PACKAGES = {}

_PACKAGE_SCAN : bool = False
_CURRENT_DIR :str = os.getcwd()
print(f"Current directory: {_CURRENT_DIR}")
_ROOT : str = os.getcwd()
_PACKAGE_DIRS = os.path.join(_ROOT, 'build-kit/packages/')
# _PACKAGE_DIRS = [os.path.join(_PACKAGE_ROOT, 'packages/')]
# _PACKAGE_DIRS = os.path.join(_PACKAGE_ROOT, 'packages/')


def search_package_dir():
    global _PACKAGES
    global _PACKAGE_DIRS
    print (f"Searching for packages in {_PACKAGE_DIRS}")
    # for pkg in os.listdir(_PACKAGE_DIRS):
    files : list = os.listdir(_PACKAGE_DIRS)
    return files


