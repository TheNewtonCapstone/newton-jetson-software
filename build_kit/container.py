#!/usr/bin/env python3
import os
import sys
import shutil
import subprocess
from typing import List, Dict, Optional
from pathlib import Path


class JetsonContainersManager:
    """Manages with the jetson-containers library."""


    @staticmethod
    def setup_jetson_containers(root_dir: Path) -> Path:
        """
        Sets up the jetson-containers dependency.

        Args:
            root_dir: Root directory where jetson-containers should be installed

        Returns:
            Path to jetson-containers installation
        """
        jetson_dir = root_dir / "lib/jetson-containers"

        # Create lib directory if it doesn't exist
        jetson_dir.parent.mkdir(parents=True, exist_ok=True)

        # Check if jetson-containers is already installed
        if not (jetson_dir / "install.sh").exists():
            print("Cloning jetson-containers repository...")
            subprocess.run(
                ["git", "clone", "https://github.com/dusty-nv/jetson-containers.git", str(jetson_dir)],
                check=True
            )

            # Run install script
            print("Running jetson-containers install script...")
            subprocess.run(
                ["bash", str(jetson_dir / "install.sh")],
                check=True
            )

        return jetson_dir


class ContainerManager:
    """Manager for building and handling containers using jetson-containers."""

    def __init__(self):
        """Initialize container manager and setup dependencies."""
        self.root_dir = Path(os.environ.get("NEWTON_ROOT", str(Path(__file__).parent)))

        # Setup jetson-containers
        self.jetson_dir = JetsonContainersManager.setup_jetson_containers(self.root_dir)

        # Add to Python path if not already there
        jetson_path = str(self.jetson_dir)
        if jetson_path not in sys.path:
            sys.path.append(jetson_path)

        # Now we can safely import jetson-containers
        from jetson_containers import (
            find_package, find_packages,
            build_container, build_containers,
            L4T_VERSION, JETPACK_VERSION
        )

        self._find_package = find_package
        self._find_packages = find_packages
        self._build_container = build_container
        self._build_containers = build_containers
        self.L4T_VERSION = L4T_VERSION
        self.JETPACK_VERSION = JETPACK_VERSION

        print(f"Using L4T version: {self.L4T_VERSION}")
        print(f"Using JetPack version: {self.JETPACK_VERSION}")

    def build(self,
              package_name: str,
              build_args: Optional[Dict[str, str]] = None,
              skip_tests: bool = False) -> str:
        """
        Build a container for the specified package.

        Args:
            package_name: Name of the package to build
            build_args: Optional build arguments
            skip_tests: Whether to skip tests

        Returns:
            Container image name

        Raises:
            RuntimeError: If build fails
        """
        try:
            args_str = {k: str(v) for k, v in (build_args or {}).items()}

            container = self._build_container(
                name='',  # Let jetson-containers choose default name
                packages=[package_name],
                build_args=args_str,
                skip_tests=['all'] if skip_tests else []
            )
            return container

        except Exception as e:
            raise RuntimeError(f"Failed to build container: {str(e)}") from e

    def list_packages(self) -> List[str]:
        """
        List available packages.

        Returns:
            List of package names

        Raises:
            RuntimeError: If listing fails
        """
        try:
            packages = self._find_packages([])  # Empty list means find all packages
            return sorted(packages.keys())
        except Exception as e:
            raise RuntimeError(f"Failed to list packages: {str(e)}") from e

    def package_info(self, package_name: str) -> Dict:
        """
        Get detailed information about a package.

        Args:
            package_name: Name of package to get info for

        Returns:
            Package information dictionary

        Raises:
            RuntimeError: If getting info fails
        """
        try:
            package = self._find_package(package_name)
            return package
        except Exception as e:
            raise RuntimeError(f"Failed to get package info: {str(e)}") from e

    def find_containers(self, search_term: str) -> List[str]:
        """
        Search for available containers matching the search term.

        Args:
            search_term: Term to search for

        Returns:
            List of matching container names

        Raises:
            RuntimeError: If search fails
        """
        try:
            packages = self._find_packages(f"*{search_term}*")
            return sorted(packages.keys())
        except Exception as e:
            raise RuntimeError(f"Failed to search containers: {str(e)}") from e

    @property
    def version(self) -> str:
        """Get the L4T and JetPack versions."""
        return f"L4T {self.L4T_VERSION}, JetPack {self.JETPACK_VERSION}"