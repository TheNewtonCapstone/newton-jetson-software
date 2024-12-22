import os

import typer
from click import prompt
from rich import print
from rich.prompt import Prompt, Confirm
from rich.console import Console
from rich.table import Table
from pathlib import Path
from typing import Optional, List
from packages import search_package_dir

app = typer.Typer()
console = Console()

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
def list():
    """Lists all available packages in a clean table format."""
    typer.echo("Listing available packages...")
    packages = search_package_dir()

    if not packages:
        print("[yellow]No packages found.[/]")
        return

    # Create a simple table with just the essential information
    table = Table(show_header=True, header_style="bold blue")
    table.add_column("Package Name", style="cyan")
    table.add_column("Path", style="green")

    # Add each package to the table, sorted alphabetically by name
    for name, info in sorted(packages.items()):
        table.add_row(name, info['path'])

    console.print(table)
@app.command()

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