import os
import subprocess
from typing import List, Dict
from pathlib import Path
from typing import List, Dict, Optional, Union, List


class Shell:
  """ """
  def __init__(self, _workspace_root :Path):
    self.workspace_root = _workspace_root
    self.executable ='/bin/bash'
    
  def run(
      self,
      cmd: Union[str, List[str]],
      env: Optional[Dict[str, str]] = None,
      cwd: Optional[Path] = None,
  ) -> subprocess.CompletedProcess:
    """Run a shell command"""
    if isinstance(cmd, list):
      cmd= " ".join(cmd)
    
    env = env or os.environ.copy()
    if env:
      env = env.update(env)
    
    return subprocess.run(cmd, env=env, cwd=cwd)