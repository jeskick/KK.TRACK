Import("env")
import subprocess
import sys
from pathlib import Path

script = Path(env.subst("$PROJECT_DIR")) / "components" / "kk" / "tools" / "gen_rx_web_page.py"
if not script.is_file():
    print("WARN: gen_rx_web_page.py not found:", script)
else:
    print("GEN web page:", script)
    subprocess.run([sys.executable, str(script)], check=True)
