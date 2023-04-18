#!/bin/bash
FILE_SIZE=`stat -c "%s" /home/nvidianx/auto-aim.log`
if ["$FILE_SIZE" -gt 1000000000];then
    rm /home/nvidianx/auto-aim.log
fi
runuser -l nvidianx -c "cd /home/nvidianx/auto-aim; export LD_LIBRARY_PATH=/usr/local/lib:/opt/MVS/lib/aarch64;./build/auto-aim >> /home/nvidianx/auto-aim.log; " &
