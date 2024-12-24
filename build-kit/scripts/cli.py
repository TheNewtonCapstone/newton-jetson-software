import os

import typer
from click import prompt
from rich import print
from rich.prompt import Prompt, Confirm
from rich.console import Console
from rich.table import Table
from typing import List

from package import Package
from build_config import BuildConfig
from package_builder import PackageBuilder
from system_info import SystemInfo, print_environment_vars, get_l4t_version
from pathlib import Path


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


    # workspace_root = get_workspace_root()
    # Initialize builder
    return

    builder = Builder(workspace_root, system_info)
    # Get available packages
    available_packages = search_package_dir()
    assert available_packages, "No packages found in workspace"

    # Determine packages to build
    if not packages:
        packages = list(available_packages.keys())

    # Create build config
    config = BuildConfig(
        arch=arch,
        build_type=build_type,
        build_tests=build_tests,
        cuda_arch=system_info.cuda_architectures
    )

    # Convert package names to Package objects
    pkg_objects = []
    for pkg_name in packages:
        assert pkg_name in available_packages, f"Package {pkg_name} not found"
        pkg_info = available_packages[pkg_name]
        pkg_objects.append(Package(
            name=pkg_name,
            path=pkg_info['path'],
            description=pkg_info.get('description', ''),
            dependencies=pkg_info.get('dependencies', []),
            build_requirements=pkg_info.get('build_requirements', {})
        ))

    # Build packages
    results = builder.build_workspace(pkg_objects, config)

    # Display results
    table = Table(show_header=True, header_style="bold blue")
    table.add_column("Package")
    table.add_column("Status")

    for pkg_name, success in results.items():
        status = "[green]Success[/]" if success else "[red]Failed[/]"
        table.add_row(pkg_name, status)

    console.print(table)
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