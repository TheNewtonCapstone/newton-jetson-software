from dataclasses import dataclass
from functools import total_ordering

@total_ordering
@dataclass
class Version:
    """
    Custom version handling class that supports:
    - Major.minor.micro version numbers (e.g., "35.2.1")
    - Comparison operations
    - String representation and parsing
    """
    major: int
    minor: int
    micro: int

    def __init__(self, version):
        if isinstance(version, Version):
            self.major = version.major
            self.minor = version.minor
            self.micro = version.micro
            return

        # Clean the version string
        version = str(version).lower().strip()
        if version.startswith('r'):
            version = version[1:]

        # Split into components
        parts = version.split('.')

        # Parse major version
        self.major = int(parts[0]) if parts else 0

        # Parse minor version
        self.minor = int(parts[1]) if len(parts) > 1 else 0

        # Parse micro version
        self.micro = int(parts[2]) if len(parts) > 2 else 0

    def __str__(self):
        if self.micro:
            return f"{self.major}.{self.minor}.{self.micro}"
        if self.minor:
            return f"{self.major}.{self.minor}"
        return str(self.major)

    def __repr__(self):
        return f"Version('{self}')"

    def __eq__(self, other):
        if not isinstance(other, Version):
            other = Version(other)
        return (self.major == other.major and
                self.minor == other.minor and
                self.micro == other.micro)

    def __lt__(self, other):
        if not isinstance(other, Version):
            other = Version(other)
        return (self.major < other.major or
                (self.major == other.major and self.minor < other.minor) or
                (self.major == other.major and
                 self.minor == other.minor and
                 self.micro < other.micro))

    def __le__(self, other):
        return self < other or self == other

    def __gt__(self, other):
        if not isinstance(other, Version):
            other = Version(other)
        return not (self <= other)

    def __ge__(self, other):
        if not isinstance(other, Version):
            other = Version(other)
        return not (self < other)

    def __hash__(self):
        return hash((self.major, self.minor, self.micro))