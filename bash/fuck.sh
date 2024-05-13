#!/bin/bash
cd /home/magician/auto-aim
LOG_DIR=$(python3 /home/magician/auto-aim/get_file_idx.py)
echo $LOG_DIR
sleep 10
# export LD_LIBRARY_PATH=/usr/local/lib:/opt/MVS/lib/64
./build/auto-aim