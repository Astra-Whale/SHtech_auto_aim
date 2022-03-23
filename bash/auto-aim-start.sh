#!/bin/bash
runuser -l nvidianx -c "cd /home/nvidianx/auto-aim; export LD_LIBRARY_PATH=/usr/local/lib:/opt/MVS/lib/aarch64;./build/auto-aim >> /home/nvidianx/auto-aim.log; " &
