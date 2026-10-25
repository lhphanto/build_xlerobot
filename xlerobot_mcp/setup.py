"""Create an isolated MCP environment and local credentials; does not start hardware."""
from pathlib import Path
import json
import os
import secrets
import subprocess
import sys
import venv
root=Path(__file__).resolve().parent
venv.EnvBuilder(with_pip=True).create(root/'.venv')
python=root/'.venv/bin/python'
subprocess.run([str(python),'-m','pip','install','-r',str(root/'requirements.txt')],check=True)
private=root/'.private'; private.mkdir(mode=0o700,exist_ok=True); private.chmod(0o700)
tokenfile=private/'token'
if not tokenfile.exists():
    fd=os.open(tokenfile,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
    with os.fdopen(fd,'w') as f: f.write(secrets.token_urlsafe(32)+'\n')
tokenfile.chmod(0o600)
# JSON string escaping is valid for these TOML basic string values.
block='[mcp_servers.xlerobot]\nurl = "http://127.0.0.1:8765/mcp"\ntool_timeout_sec = 60\ndefault_tools_approval_mode = "prompt"\nhttp_headers = { Authorization = '+json.dumps('Bearer '+tokenfile.read_text().strip())+' }\n'
config=private/'codex-config.toml'; config.write_text(block); config.chmod(0o600)
print('Setup complete. Credential/config files created under .private (do not share or commit).')
print('No service started and no Codex settings changed.')
