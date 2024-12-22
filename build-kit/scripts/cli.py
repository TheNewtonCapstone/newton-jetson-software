from email.policy import default
from re import search

import typer
from click import prompt
from rich import print
from rich.prompt import Prompt, Confirm
from pathlib import Path
from typing import Optional, List
from packages import search_package_dir
app = typer.Typer()

# TODO - add support for creating new workspaces
@app.command()
def init():
    """Initialize a new robot workspace"""
    pass

@app.command()
def build(
    packages: List[str] = typer.Option(
        default=[],
        prompt="Which packages would you like to build?",
        help="Packages to build"
    ),
    arch: str = typer.Option(
        default="jetson",
        prompt="What is the target architecture?",
        help="Target architecture [jetson/x86_64]"),
):
    """Build specific packages"""
    pass
    # for pkg in packages:
    #     pass
        # print(f"Building {pkg}...")
        # Build logic here

# TODO - add support for creating new modules
@app.command()
def create():
    pass

@app.command()
def list(
        packages: str = typer.Option(
            default="",
            help="List available packages")
) -> None:
    typer.echo("Listing available packages...")
    search_package_dir()
    """List available packages
    # :param packages:
    """

@app.command()
def test(
    test_type: str = typer.Option("unit", help="Test type [unit/integration/all]"),
    hardware: bool = typer.Option(False, help="Run hardware tests")
):
    """Run platform tests"""
    print(f"Running {test_type} tests...")
    # Test logic here

if __name__ == "__main__":
    app()