import os
import sys
import re


here = os.path.dirname(__file__)
log_path = os.path.join(here, "logs", "autostart.log")
LD_LIBRARY_PATH = os.environ.get("LD_LIBRARY_PATH")
start_bash = f"""#!/bin/bash
export LD_PRELOAD=/lib/aarch64-linux-gnu/libasan.so.6
export ASAN_OPTIONS=detect_leaks=1:abort_on_error=1
mkdir -p /root/auto-aim/./logs
chmod 0777 /root/auto-aim/./logs
touch /root/auto-aim/./logs/autostart.log
chmod 0666 /root/auto-aim/./logs/autostart.log
FILE_SIZE=$(stat -c "%s" /root/auto-aim/./logs/autostart.log)
if [ "$FILE_SIZE" -gt 1000000000 ]; then
    rm /root/auto-aim/./logs/autostart.log
    touch /root/auto-aim/logs/autostart.log
fi
ulimit -c unlimited
mkdir -p /var/coredump
chmod 777 /var/coredump
echo "/var/coredump/core-%e-%p-%t" | tee /proc/sys/kernel/core_pattern
export LD_LIBRARY_PATH=/usr/local/lib:/opt/MVS/lib/aarch64:/usr/local/lib
cd /root/auto-aim || exit 1
echo "Waiting for 20 seconds before starting auto-aim..." >> ./logs/autostart.log
sleep 20
./build/auto-aim >> /root/auto-aim/logs/autostart.log 2>&1



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
LimitCORE=infinity
Type=Simple
 
Environment="LD_PRELOAD=/lib/aarch64-linux-gnu/libasan.so.6"
Environment="ASAN_OPTIONS=detect_leaks=1:abort_on_error=1:log_path=/tmp/asan.log"

ExecStart=/root/auto-aim/bash/auto-aim-start.sh
ExecStop=/root/auto-aim/bash/auto-aim-stop.sh
PrivateTmp=true
 
[Install]
WantedBy=multi-user.target
Alias=auto-aim.service
""")
print(f"Service Installed! /lib/systemd/system/auto-aim.service ")
os.system("systemctl daemon-reload")
