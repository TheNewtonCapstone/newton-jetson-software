from typing import Dict, List, Optional
from dataclasses import dataclass
from pathlib import Path
import subprocess
import logging
import os

from system_info import SystemInfo
from shell import Shell
from system_info import *

from dataclasses import dataclass
from typing import List
from pathlib import Path

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

# TODO - refactor this to use the Shell class, im only cutting corners for now
_NEWLINE_=" \\\n"  

@dataclass
class Package:
    """Build configuration for a package"""
    name: str
    workspace: Path
    dockerfile: Path
    
    # Container settings
    container_name: str
    base: str 
    tag: Optional[str] = None
    
    # Build configuration
    build_args: Dict[str, str] = field(default_factory=dict)
    build_flags: str = ""
    prefix: str = ""
    postfix: str = ""
    
    # Dependencies and requirements
    dependencies: List[str] = field(default_factory=list)
    build_requirements: Dict[str, str] = field(default_factory=dict)
    
    install_script: Optional[str] = None
    
    # Logging
    log_file: str = ""

    
    def __post_init__(self):
        """Convert string paths to Path objects"""
        if isinstance(self.workspace, str):
            self.workspace = Path(self.workspace)
        if isinstance(self.dockerfile, str):
            self.dockerfile = Path(self.dockerfile)
        if isinstance(self.install_script, str):
            self.install_script = Path(self.install_script) 
        if not self.install_script:
            self.install_script = self.workspace / f"{self.name}_install.sh"

class PackageBuilder:
    """

    """

    def __init__(self, workspace_root: Path,system_info: SystemInfo):
        self.package_dir = Path(workspace_root)
        self.system_info = system_info
        assert self.package_dir.exists(), f"Workspace root does not exist: {self.workspace_root}"
    

    def validate_package(self, package: Package) -> bool:
        """Validate package requirements against system info"""
        # TODO - Check CUDA version requirements if specified
        # TODO - Check JetPack version if specified
        # TODO - Check L4T version if specified
        return True

    def build_package(self, package: Package) -> subprocess.CompletedProcess:
        """Build a single package with given configuration"""
        assert self.validate_package(package), f"Package validation failed for {package.name}"
        if not package.base:
            package.base = get_l4t_base()
        
        # TODO : resolve dependencies
        # TODO : Find packages and make sure all of them are there before building any of them
        # TODO : introduce a nice parser for the build cmd
        
        cmd = f"{self.add_sudo_prefix()}DOCKER_BUILDKIT=0 docker build --network=host"
        cmd += f" --tag {package.container_name}" 
        cmd += f" -f {package.dockerfile}" 
        cmd += f" {package.workspace}" 
        # cmd += f"2>&1 | tee {package.log_file + '.txt'}" + "; exit ${PIPESTATUS[0]}"  
        env = os.environ.copy()
        env['DOCKER_BUILDKIT'] = '0'

        with open(package.install_script, 'w') as cmd_file:
            cmd_file.write('#!/bin/bash\n')
            cmd_file.write(cmd + '\n')
        print(f'Running cmd {cmd}') 
        subprocess.run(['chmod', '+x', package.install_script], check=True)
        install_script = str(package.install_script)
        print(f'Running install script {install_script}')
        subprocess.run(['bash', install_script], check=True, env=env)
    
    
    def resolve_dependencies(self, package: Package) -> List[Package]:
        """Resolve dependencies for a package"""
        pass

    def add_sudo_prefix(self, group:str ='docker') -> str:
        """Add sudo prefix if required"""
        return "sudo " if os.geteuid() != 0 else ""
