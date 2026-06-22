Import("env")
import subprocess
import sys
from pathlib import Path

project = Path(env.subst("$PROJECT_DIR"))
repo = project.parents[2]
script = repo / "components" / "kk" / "tools" / "gen_rx_web_page.py"
if not script.is_file():
    print("WARN: gen_rx_web_page.py not found:", script)
else:
    print("GEN web page:", script)
    subprocess.run([sys.executable, str(script)], check=True)

verify = repo / "components" / "kk" / "tools" / "verify_kk_copies.py"
if verify.is_file():
    print("VERIFY kk copies:", verify)
    subprocess.run([sys.executable, str(verify)], check=True)
