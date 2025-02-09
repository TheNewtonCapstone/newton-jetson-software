import os
import platform
import subprocess
import shutil
import typer
from click import prompt
from rich import print
from rich.prompt import Prompt, Confirm
from rich.console import Console
from rich.table import Table
from typing import List, Optional
from pathlib import Path
import select



app = typer.Typer()
ctn_app = typer.Typer()
pkg_app = typer.Typer()

app.add_typer(ctn_app, name="ctn", help="Actions related with containers")
app.add_typer(pkg_app, name="pkg", help="Actions related with packages")
console = Console()
ROOT_DIR = ""

pkg_app.command("build")
def build_package(
        name: str = typer.Argument(..., help="Name of the container to build"),
):
    """Build ros package """
    if name == "motor_driver":
        pass
    elif name == "odrive_can":
        pass
    console.print("Built")

pkg_app.command("source")
def source_package():
    pass

@app.command("clean")
def clean_workspace():
    pass
@app.command()
def init():
    """Initialize a new robot workspace"""
    pass



@app.command()
def list_packages():
    """List all available packages"""
    try:
      pass
    except Exception as e:
        console.print(f"[red]Error listing packages: {str(e)}[/red]")


@app.command()
def create():
    pass


@app.command()
def search(
        term: str = typer.Argument(..., help="Search term for finding packages")
):
    """Search for available packages"""
    try:
      pass

    except Exception as e:
        console.print(f"[red]Error searching packages: {str(e)}[/red]")



@ctn_app.command("list")
def list_containers():
    pass


@ctn_app.command("run")
def run_container(
    name: str = typer.Argument(..., help="Name of the container to run"),
    command: Optional[List[str]] = typer.Option(None, "--cmd", help="Command to run in the container"),
    interactive: bool = typer.Option(True, "--interactive", "-i", help="Run container in interactive mode"),
    workspace_source: bool = typer.Option(True, "--workspace-source", help="Source workspace setup.bash"),
    use_display: bool = typer.Option(True, "--display", help="Enable X11 display forwarding"),
    host_network: bool = typer.Option(True, "--host-network", help="Use host network")
):
    """Run a container with hardware access and proper ROS2 environment setup"""
    try:
        workspace_root = get_workspace_root()
        cmd = [
            "docker", "run", "-it",
            "--net=host",
            "-e", f"DISPLAY={os.getenv('DISPLAY', ':0')}",
            "-v", "/dev:/dev",
            "--device-cgroup-rule=c *:* rmw",
            "--device=/dev",
            "-v", f"{workspace_root}:{workspace_root}",
            "-w", str(workspace_root),
            name,
            "/bin/bash"
        ]
        os.execvp(cmd[0], cmd)
    except Exception as e:
        console.print(f"[red]Error building package: {str(e)}[/red]")
        
    
@ctn_app.command("build")
def build_container(
        name: str = typer.Argument(..., help="Name of the container to build"),
        cache: bool = typer.Option(False, help="Use cache when building the container"),
        build_args: Optional[List[str]] = typer.Option(None, "--build-arg", help="Build arguments in format KEY=VALUE")
):
    """Build a container package"""
    try:
        cmd = ["docker", "build"]
        if build_args:
            for arg in build_args:
              cmd.extend(["--build-arg", arg])
        if not cache:
            cmd.append("--no-cache")

        # pass in build arguments

        default_args = {
            "ROOT": f"{str(get_workspace_root())}",
            "HAS_CUDA": str(has_cuda())
        }

        for key, value in default_args.items():
            cmd.extend(["--build-arg", f"{key}={value}"])
            
        workspace_root = get_workspace_root()
        arch = platform.machine()

        if arch == "aarch64":
          docker_file_name = "Dockerfile.aarch_64"
          console.print("Docker", docker_file_name)
        elif arch == "x86_64":
          docker_file_name = "Dockerfile.x86_64"
        else:
          raise Exception(f"Unsupported architecture: {arch}")
        path_to_dockerfile = Path(workspace_root, "docker/", docker_file_name)


        if not path_to_dockerfile.exists():
            raise Exception(f"Could not find Dockerfile: {path_to_dockerfile}")

        
        tag=f"{name}:{arch}"
        cmd.extend(["-t", name])
        cmd.extend(["-t", tag])
        
        # context
        docker_context = str(Path(workspace_root, "docker/"))
        cmd.extend(["-f", str(path_to_dockerfile)])
        cmd.append(docker_context)

        console.print(f"[yellow]Running command: {' '.join(cmd)}[/yellow]")
        process = subprocess.Popen(
            cmd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            cwd=workspace_root
        )
        while True:
            ready, _, _ = select.select([process.stdout, process.stderr], [], [], 1)  # Timeout of 1s
            for stream in ready:
                line = stream.readline().strip()
                if line:
                    console.print(line)

            exit_condition = process.poll() is not None
            if exit_condition:
                break
            # if error:
            #     console.print(f"[red]{error.strip()}[/red]")
        
            # console.print(f"{output.strip()}")
        return_code = process.poll()
        if return_code != 0:
            raise Exception(f"Error building container: {process.stderr.read()}")
        else:
            console.print(f"[green]Successfully built container: {name}[/green]")


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


def has_cuda():
  return 1
@app.command()
def info(
        package_name: str = typer.Argument(..., help="Name of the package to get info about")
):
    """Get detailed information about a package"""
    try:

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