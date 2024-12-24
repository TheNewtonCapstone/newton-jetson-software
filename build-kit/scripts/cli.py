import os

import typer
from click import prompt
from rich import print
from rich.prompt import Prompt, Confirm
from rich.console import Console
from rich.table import Table
from typing import List
from platform import platform
from pathlib import Path

from system_info import *
from package_builder import PackageBuilder, Package




app = typer.Typer()
console = Console()
ROOT_DIR = ""

# TODO - add support for creating new workspaces
@app.command()
def init():
    """Initialize a new robot workspace"""
    pass


# TODO - add support for creating new modules
@app.command()
def create():
    pass


@app.command()
def build(
        packages: List[str] = typer.Option(
            default=None,
            help="Packages to build (default: all)"
        ),
        arch: str = typer.Option(
            default="jetson",
            help="Target architecture [jetson/x86_64]"
        ),
        build_type: str = typer.Option(
            default="Release",
            help="Build type [Debug/Release]"
        ),
        build_tests: bool = typer.Option(
            default=False,
            help="Build tests"
        )
):
    """Build packages in the workspace"""
    # Get workspace and system info
    # get_l4t_version()
    print_environment_vars()
    get_l4t_version()
    system_info = SystemInfo(
        l4t_version=get_l4t_version(),
        jetpack_version=get_jetpack_version(),
        cuda_version=get_cuda_version(),
        cuda_architectures=[53, 62, 72],
        system_arch=platform.machine(),
        python_version=Version(platform.python_version()),
        lsb_release=get_lsb_release().release,
        lsb_codename=get_lsb_release().codename,
        lsb_id=get_lsb_release().id)
    print(system_info)
    
    ROOT_DIR = get_workspace_root()
    print(f"ROOT_DIR: {ROOT_DIR}")
    builder = PackageBuilder(workspace_root=ROOT_DIR, system_info=system_info)

    ros_package = Package(
        name="ros",
        workspace=ROOT_DIR / "build-kit/packages/ros",
        dockerfile=ROOT_DIR / "build-kit/packages/ros/Dockerfile",
        container_name="newton",
        base="newton",
        tag="ros",
        build_args=['--no-cache'],
        dependencies=[],
        log_file="ros_build.log",
        build_requirements={},
    )
    builder.build_package(ros_package)

    





    # workspace_root = get_workspace_root()
    # Initialize builder
    return None
app.command()
def set_root_dir(
    root_dir: str = typer.Argument(
        default=os.getcwd(),
        help="Root directory for the workspace"
    )

):
    """Set the root directory for the workspace"""
    global ROOT_DIR
    ROOT_DIR = root_dir
    print(f"Setting root directory to {ROOT_DIR}")
@app.command()
def test(
    test_type: str = typer.Option("unit", help="Test type [unit/integration/all]"),
    hardware: bool = typer.Option(False, help="Run hardware tests")
):
    """Run platform tests"""
    print(f"Running {test_type} tests...")
    # Test logic here


def get_workspace_root() -> Path:
    """Get the workspace root directory"""
    if 'WORKSPACE_ROOT' in os.environ:
        return Path(os.environ['WORKSPACE_ROOT'])

    # Look for workspace marker file in current or parent directories
    current = Path.cwd()
    while current != current.parent:
        if (current / '.workspace').exists():
            return current
        current = current.parent

    # Default to current directory
    return Path.cwd()
if __name__ == "__main__":
    app()