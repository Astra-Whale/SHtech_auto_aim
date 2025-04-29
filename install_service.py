import os
import sys
import re


here = os.path.dirname(__file__)
log_path = os.path.join(here, "logs", "autostart.log")
LD_LIBRARY_PATH = os.environ.get("LD_LIBRARY_PATH")
start_bash = f"""#!/bin/bash
mkdir -p {os.path.join(here, "logs")}
chmod 0777 {os.path.join(here, "logs")}
touch {log_path}
chmod 0666 {log_path}
FILE_SIZE=$(stat -c "%s" {log_path})
if [ "$FILE_SIZE" -gt 1000000000 ]; then
    rm {log_path}
fi
runuser -l {os.getlogin()} -c "cd {here}; export LD_LIBRARY_PATH={LD_LIBRARY_PATH};./build/auto-aim >> {log_path} 2>&1" &

"""
print(start_bash)
with open(os.path.join(here, "bash", "auto-aim-start.sh"), "w") as f:
    f.write(start_bash)
os.chmod(os.path.join(here, "bash", "auto-aim-start.sh"), 0o777)

if os.getlogin() != "root":
    home_dir = os.path.join("/home", os.getlogin())
else:
    home_dir = "/root"
bashrc_path = os.path.join(home_dir, ".bashrc")
print(home_dir)

# Read the current content
with open(bashrc_path, 'r') as f:
    lines = f.readlines()

# Pattern to match autoaim aliases
pattern = re.compile(r'^\s*alias\s+autoaim-\w+\s*=\s*.*$')

# Filter out matching lines
bashrc = "".join([line for line in lines if not pattern.match(line)])
with open(bashrc_path, 'w') as f:
    f.write(bashrc.removesuffix("\n"))
    f.write(f"""
alias autoaim-stop="sudo systemctl stop auto-aim"
alias autoaim-start="sudo systemctl start auto-aim"
alias autoaim-enable="sudo systemctl enable auto-aim"
alias autoaim-disable="sudo systemctl disable auto-aim"
alias autoaim-status="systemctl status auto-aim"
alias autoaim-help="{os.path.join(here, "bash", "auto-aim-help.sh")}"
""")

with open("/lib/systemd/system/auto-aim.service", "w") as f:
    f.write(f"""[Unit]
Description=auto-aim service
 
[Service]
Type=forking
 
ExecStart={here}/bash/auto-aim-start.sh
ExecStop={here}/bash/auto-aim-stop.sh
PrivateTmp=true
 
[Install]
WantedBy=multi-user.target
Alias=auto-aim.service
""")
print(f"Service Installed! /lib/systemd/system/auto-aim.service ")
os.system("systemctl daemon-reload")
