import os
import platform
from typing import List
from dataclasses import dataclass
import json

from rich.console import Console
from rich.table import Table
from rich.text import Text
from packaging.version import Version

@dataclass
class SystemInfo:
    """System information container"""
    l4t_version: Version
    jetpack_version: Version
    cuda_version: Version
    cuda_architectures: List[int]
    system_arch: str
    python_version: Version
    lsb_release: str
    lsb_codename: str
    lsb_id: str
@dataclass(frozen=True)
class LsbInfo:
    """Linux Standard Base (LSB) information container"""
    release: str
    codename: str
    id: str

def get_l4t_version(path: str='/etc/nv_tegra_release') -> Version:
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

    if platform.machine() != 'aarch64':
        print(f"L4T_VERSION isn't supported on {platform.machine()} architecture (aarch64 only)")
        return Version('0.0.0')

    assert os.path.isfile(path), f"L4T_VERSION file not found at '{path}'"
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



def get_lsb_release() -> LsbInfo:
    """ Returns theLSB release string """
    try:
        with open('/etc/lsb-release') as file:
            lsb_info = {}
            for line in file:
                key, value = line.strip().split('=')
                lsb_info[key] = value.strip('"')
            return LsbInfo(
                release=lsb_info['DISTRIB_RELEASE'],
                codename=lsb_info['DISTRIB_CODENAME'],
                id=lsb_info['DISTRIB_ID']
            )
    except FileNotFoundError:
        return LsbInfo(release='0.0', codename='UNKNOWN', id='UNKNOWN')

def get_cuda_version() -> Version:
    """ Returns the CUDA version installed on the system """
    try:
        with open('/usr/local/cuda/version.json') as file:
            version_info = json.load(file)
            parts = version_info['cuda']['version'].split('.')
            if len(parts) == 3:
                return Version(version_info['cuda']['version'])
            else:
                return Version('0.0.0')

    except FileNotFoundError:
        return Version('0.0.0')


# TODO : add in documentation that this is only for r32.5.0 and above
def get_l4t_base(l4t_version=get_l4t_version()) -> Version:
    """
    Return docker base image for L4T version
    """
    if l4t_version.major >= 36:
        return "nvcr.io/nvidia/l4t-base:r36.0.0"
    else :
        return "nvcr.io/nvidia/l4t-base:r32.5.0"

def arch_is_l4t_compatible(arch: str) -> bool:
    """ Returns True if the architecture is compatible with L4T (aarch64) """
    return platform.machine() == 'aarch64'

def get_jetpack_version(l4t_version=get_l4t_version(), default='6.1'):
    """
    Returns the version of JetPack (based on the L4T version)
    https://github.com/rbonghi/jetson_stats/blob/master/jtop/core/jetson_variables.py
    """

    if not arch_is_l4t_compatible(platform.machine()):
        print(f"JetPack version is only available on L4T compatible systems")
        return Version('0.0.0')

    if not isinstance(l4t_version, Version):
        l4t_version = Version(l4t_version)

    NVIDIA_JETPACK = {
        # -------- JETPACK 6--------
        "36.4.0": "6.1 GA",
        "36.3.0": "6.0 GA",
        "36.2.0": "6.0 DP",
        "36.0.0": "6.0 EA",

        # -------- Jetpack 5--------
        "35.4.1": "5.1.2",
        "35.3.1": "5.1.1",
        "35.3.0": "5.1.1 PRE",
        "35.2.1": "5.1",
        "35.1.0": "5.0.2 GA",
        "34.1.1": "5.0.1 DP",
        "34.1.0": "5.0 DP",
        "34.0.1": "5.0 PRE-DP",

        # -------- Jetpack 4--------
        "32.7.4": "4.6.4",
        "32.7.3": "4.6.3",
        "32.7.2": "4.6.2",
        "32.7.1": "4.6.1",
        "32.6.1": "4.6",
        "32.5.2": "4.5.1",
        "32.5.1": "4.5.1",
        "32.5.0": "4.5",
        "32.5": "4.5",
        "32.4.4": "4.4.1",
        "32.4.3": "4.4",
        "32.4.2": "4.4 DP",
        "32.3.1": "4.3",
        "32.2.3": "4.2.3",
        "32.2.1": "4.2.2",
        "32.2.0": "4.2.1",
        "32.2": "4.2.1",
        "32.1.0": "4.2",
        "32.1": "4.2",
        "31.1.0": "4.1.1",
        "31.1": "4.1.1",
        "31.0.2": "4.1",
        "31.0.1": "4.0",


        # -------- OLD Jetpack--------
        "28.4.0": "3.3.3",
        "28.2.1": "3.3 | 3.2.1",
        "28.2.0": "3.2",
        "28.2": "3.2",
        "28.1.0": "3.1",
        "28.1": "3.1",
        "27.1.0": "3.0",
        "27.1": "3.0",
        "24.2.1": "3.0 | 2.3.1",
        "24.2.0": "2.3",
        "24.2": "2.3",
        "24.1.0": "2.2.1 | 2.2",
        "24.1": "2.2.1 | 2.2",
        "23.2.0": "2.1",
        "23.2": "2.1",
        "23.1.0": "2.0",
        "23.1": "2.0",
        "21.5.0": "2.3.1 | 2.3",
        "21.5": "2.3.1 | 2.3",
        "21.4.0": "2.2 | 2.1 | 2.0 | 1.2 DP",
        "21.4": "2.2 | 2.1 | 2.0 | 1.2 DP",
        "21.3.0": "1.1 DP",
        "21.3": "1.1 DP",
        "21.2.0": "1.0 DP",
        "21.2": "1.0 DP",
    }

    for key in NVIDIA_JETPACK:
        if Version(key) == l4t_version:
            return Version(NVIDIA_JETPACK[key].split(' ')[0])
    return Version(default)


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

