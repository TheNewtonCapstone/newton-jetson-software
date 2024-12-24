from typing import Dict, List, Optional
from dataclasses import dataclass
from pathlib import Path
import subprocess
import logging
import os

from system_info import SystemInfo
from version import Version
from package import Package
from build_config import BuildConfig

class PackageBuilder:
    """Main builder class for handling package builds"""

    def __init__(self, workspace_root: Path, system_info: SystemInfo):
        self.workspace_root = Path(workspace_root)
        self.system_info = system_info
        self.build_dir = self.workspace_root / "build"
        self.install_dir = self.workspace_root / "install"

        # Validate workspace structure
        assert self.workspace_root.exists(), f"Workspace root does not exist: {self.workspace_root}"
        self.build_dir.mkdir(exist_ok=True)
        self.install_dir.mkdir(exist_ok=True)

    def validate_package(self, package: Package) -> bool:
        """Validate package requirements against system info"""
        # Check CUDA version requirements if specified
        if 'cuda' in package.build_requirements:
            required_cuda = Version(package.build_requirements['cuda'])
            assert self.system_info.cuda_version >= required_cuda, \
                f"CUDA version {required_cuda} required, found {self.system_info.cuda_version}"

        # Check JetPack version if specified
        if 'jetpack' in package.build_requirements:
            required_jp = Version(package.build_requirements['jetpack'])
            assert self.system_info.jetpack_version >= required_jp, \
                f"JetPack version {required_jp} required, found {self.system_info.jetpack_version}"

        # Check L4T version if specified
        if 'l4t' in package.build_requirements:
            required_l4t = Version(package.build_requirements['l4t'])
            assert self.system_info.l4t_version >= required_l4t, \
                f"L4T version {required_l4t} required, found {self.system_info.l4t_version}"

        return True

    def build_package(self, package: Package, config: BuildConfig) -> bool:
        """Build a single package with given configuration"""
        assert self.validate_package(package), f"Package validation failed for {package.name}"

        pkg_build_dir = self.build_dir / package.name
        pkg_build_dir.mkdir(exist_ok=True)

        # Set up build environment
        env = os.environ.copy()
        env.update({
            'CUDA_ARCH_LIST': ';'.join(map(str, config.cuda_arch or self.system_info.cuda_architectures)),
            'CMAKE_INSTALL_PREFIX': str(self.install_dir),
            'BUILD_TESTING': 'ON' if config.build_tests else 'OFF',
            'CMAKE_BUILD_TYPE': config.build_type,
        })

        # Configure
        result = subprocess.run(
            ['cmake', str(package.path),
             f'-DCMAKE_INSTALL_PREFIX={self.install_dir}',
             f'-DCMAKE_BUILD_TYPE={config.build_type}'],
            cwd=pkg_build_dir,
            env=env,
            capture_output=True,
            text=True
        )
        assert result.returncode == 0, f"CMake configure failed for {package.name}: {result.stderr}"

        result = subprocess.run(
            ['cmake', '--build', '.', '-j'],
            cwd=pkg_build_dir,
            env=env,
            capture_output=True,
            text=True
        )
        assert result.returncode == 0, f"CMake build failed for {package.name}: {result.stderr}"

        # Install
        result = subprocess.run(
            ['cmake', '--install', '.'],
            cwd=pkg_build_dir,
            env=env,
            capture_output=True,
            text=True
        )
        assert result.returncode == 0, f"CMake install failed for {package.name}: {result.stderr}"

        return True

    def build_workspace(self, packages: List[Package], config: BuildConfig) -> Dict[str, bool]:
        """Build multiple packages in correct dependency order"""
        results = {}
        built_packages = set()

        def build_with_deps(pkg: Package) -> bool:
            if pkg.name in built_packages:
                return results[pkg.name]

            # Build dependencies first
            for dep in pkg.dependencies:
                assert dep in [p.name for p in packages], \
                    f"Dependency {dep} not found for package {pkg.name}"
                if dep not in built_packages:
                    dep_pkg = next(p for p in packages if p.name == dep)
                    assert build_with_deps(dep_pkg), \
                        f"Failed to build dependency {dep} for package {pkg.name}"

            # Build package
            success = self.build_package(pkg, config)
            results[pkg.name] = success
            if success:
                built_packages.add(pkg.name)
            return success

        # Build all packages
        for package in packages:
            build_with_deps(package)

        return results