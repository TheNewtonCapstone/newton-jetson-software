import os
import platform
from dataclasses import dataclass

from rich.console import Console
from rich.table import Table
from rich.text import Text

from version import Version

def get_l4t_version(path: str='etc/nv_tegra_release') -> Version:
    """
    Returns the L4T_VERSION in a packaging.version.Version object
    Which can be compared against other version objects:  https://packaging.pypa.io/en/latest/version.html
    You can also access the version components directly.  For example, on L4T R35.3.1:

        version.major == 35
        version.minor == 3
        version.micro == 1

    The L4T_VERSION will either be parsed from /etc/nv_tegra_release or the $L4T_VERSION environment variable.
    """
    l4t_release_prefix = '# R'
    l4t_release_suffix = ' (release)'

    assert platform.machine() == 'aarch64', f"L4T_VERSION isn't supported on {platform.machine()} architecture (aarch64 only)"
    assert os.path.isfile(path), f"L4T_VERSION file doesn't exist:  {path}"
    with open(path) as file:
        line = file.readline()
    parts = [part.strip() for part in line.split(',')]

    # inspect the release format
    l4t_release = parts[0]
    assert l4t_release.startswith(l4t_release_prefix) and l4t_release.endswith(l4t_release_suffix), f"L4T release string is invalid or in unexpected format:  '{l4t_release}'"
    l4t_release = l4t_release[len(l4t_release_prefix):-len(l4t_release_suffix)]

    #parse the revision
    l4t_revision = parts[1]
    l4t_revision_prefix = 'REVISION: '
    assert l4t_revision.startswith(l4t_revision_prefix), f"L4T revision '{l4t_revision}' doesn't start with expected prefix '{l4t_revision_prefix}'"

    l4t_revision = l4t_revision[len(l4t_revision_prefix):]
    # print("[yellow] L4T REVISION [/]")
    # print(f"l4t_release: {l4t_release}")
    return Version(f'{l4t_release}.{l4t_revision}')


def print_environment_vars(env_vars=None):
    """
    Print environment variables in a nicely formatted table
    Args:
        env_vars (dict): Dictionary of environment variables. If None, uses os.environ
    """
    if env_vars is None:
        env_vars = dict(os.environ)

    print(f"Machine : {platform.machine()}")
    # Create console and table
    console = Console()
    table = Table(show_header=True, header_style="bold magenta")

    # Add columns
    table.add_column("Variable", style="cyan", no_wrap=True)
    table.add_column("Value", style="green")

    # Sort environment variables alphabetically
    sorted_vars = sorted(env_vars.items(), key=lambda x: x[0].lower())

    # Process and add rows
    for var, value in sorted_vars:
        # Handle PATH-like variables specially (split by :)
        if any(path_var in var.upper() for path_var in ['PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH']):
            paths = value.split(':')
            formatted_value = Text()
            for i, path in enumerate(paths):
                if i > 0:
                    formatted_value.append('\n')
                formatted_value.append(path, style="green")
        else:
            formatted_value = value

        table.add_row(var, formatted_value)

    # Print the table
    console.print(table)

@dataclass
class SystemInfo:
    """System information container"""
    l4t_version: Version
    # jetpack_version: Version
    # cuda_version: Version
    # cuda_architectures: List[int]
    # system_arch: str
    # python_version: Version
    # lsb_release: str
    # lsb_codename: str
    # lsb_id: str

