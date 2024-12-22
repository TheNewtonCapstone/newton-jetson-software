#!/usr/bin/env python3
import os
from pathlib import Path

_PACKAGES = {}

_PACKAGE_SCAN : bool = False
_CURRENT_DIR :str = os.getcwd()
print(f"Current directory: {_CURRENT_DIR}")
ROOT_DIR : str = os.getcwd()
PACKAGES_DIR= os.path.join(ROOT_DIR, 'build-kit/packages/')
# TODO  - create a package Manager class and Package @dataclass
def is_valid_package(path : Path) -> bool:
    packages_files = ['Dockerfile']
    for file in packages_files:
        no_file : bool = not((path/file).exists())
        if no_file:
            return False
    return True

def search_package_dir() -> dict:
    global PACKAGES_DIR
    print (f"Searching for packages in {PACKAGES_DIR}")

    packages = {}
    package_path : Path = Path(PACKAGES_DIR)
    if not package_path.exists():
        print(f"Package directory not found: {package_path}")
        return packages
    for item in package_path.glob('**/*'):
        if item.is_dir() and is_valid_package(item):
            print(f"Found package: {item}")
            path = item.relative_to(package_path)
            package_name = str(path).replace('/','-')

            packages[package_name] = {
                'path': str(item),
                'name': package_name,
                'description': 'TODO - add description',
                'dependencies': []
            }
    global _PACKAGES
    _PACKAGES = packages
    return packages


