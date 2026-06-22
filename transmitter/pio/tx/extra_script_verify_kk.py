Import("env")
import subprocess
import sys
from pathlib import Path

project = Path(env.subst("$PROJECT_DIR"))
repo = project.parents[2]
verify = repo / "components" / "kk" / "tools" / "verify_kk_copies.py"
if verify.is_file():
    print("VERIFY kk copies:", verify)
    subprocess.run([sys.executable, str(verify)], check=True)
