from dataclasses import dataclass
from typing import List

@dataclass
class BuildConfig:
    """Build configuration settings"""
    arch: str
    build_type: str = "Release"
    build_tests: bool = False
    cuda_arch: List[int] = None

    def __post_init__(self):
        assert self.arch in ["jetson", "x86_64"], f"Unsupported architecture: {self.arch}"
        assert self.build_type in ["Debug", "Release"], f"Unsupported build type: {self.build_type}"
