from dataclasses import dataclass
from typing import List, Dict
@dataclass
class Package:
    """Package information container"""
    name: str
    path: str
    description: str
    dependencies: List[str]
    build_requirements: Dict[str, str]
