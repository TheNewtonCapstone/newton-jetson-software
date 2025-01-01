import os

import typer
from click import prompt
from rich import print
from rich.prompt import Prompt, Confirm
from rich.console import Console
from rich.table import Table
from typing import List, Optional
from platform import platform
from pathlib import Path
from .container import ContainerManager


app = typer.Typer()
console = Console()
ctn_manager = ContainerManager()
ROOT_DIR = ""

# TODO - add support for creating new workspaces
@app.command()
def init():
    """Initialize a new robot workspace"""
    pass


@app.command()
def list_packages():
    """List all available packages"""
    try:
        packages = ctn_manager.list_packages()

        table = Table(title="Available Packages")
        table.add_column("Package Name", style="cyan")

        for package in packages:
            table.add_row(package)

        console.print(table)

    except Exception as e:
        console.print(f"[red]Error listing packages: {str(e)}[/red]")

# TODO - add support for creating new modules
@app.command()
def create():
    pass


@app.command()
def search(
        term: str = typer.Argument(..., help="Search term for finding packages")
):
    """Search for available packages"""
    try:
        results = ctn_manager.find_containers(term)

        table = Table(title=f"Search Results for: {term}")
        table.add_column("Package Name", style="cyan")

        for result in results:
            table.add_row(result)

        console.print(table)

    except Exception as e:
        console.print(f"[red]Error searching packages: {str(e)}[/red]")


@app.command()
def build(
        package_name: str = typer.Argument(..., help="Name of the package to build"),
        skip_tests: bool = typer.Option(False, "--skip-tests", help="Skip package tests"),
        build_args: Optional[List[str]] = typer.Option(None, "--build-arg", help="Build arguments in format KEY=VALUE")
):
    """Build a container package"""
    try:
        # Convert build args list to dictionary
        build_args_dict = {}
        if build_args:
            for arg in build_args:
                key, value = arg.split('=', 1)
                build_args_dict[key] = value

        console.print(f"Building package [cyan]{package_name}[/cyan]...")
        if build_args_dict:
            console.print(f"Build arguments: {build_args_dict}")

        container = ctn_manager.build(
            package_name=package_name,
            build_args=build_args_dict,
            skip_tests=skip_tests
        )

        console.print(f"\n[green]Successfully built container: [blue]{container}[/blue][/green]")

    except Exception as e:
        console.print(f"[red]Error building package: {str(e)}[/red]")


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


@app.command()
def info(
        package_name: str = typer.Argument(..., help="Name of the package to get info about")
):
    """Get detailed information about a package"""
    try:
        info = ctn_manager.package_info(package_name)

        table = Table(title=f"Package Information: {package_name}")
        table.add_column("Property", style="cyan")
        table.add_column("Value", style="green")

        for key, value in info.items():
            if isinstance(value, (list, dict)):
                value = str(value)
            table.add_row(str(key), str(value))

        console.print(table)

    except Exception as e:
        console.print(f"[red]Error getting package info: {str(e)}[/red]")
if __name__ == "__main__":
    app()