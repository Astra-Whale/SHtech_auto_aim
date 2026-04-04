#!/bin/bash
set -u

LOG_FILE="/root/auto-aim/logs/autostart.log"
TIMEOUT_SECONDS=8

log() {
    if [ -d "/root/auto-aim/logs" ]; then
        echo "$(date '+%F %T') [auto-aim-stop] $1" >> "$LOG_FILE"
    fi
}

pids=$(pgrep -x auto-aim || true)
if [ -z "$pids" ]; then
    log "no auto-aim process found"
    exit 0
fi

log "sending SIGTERM to auto-aim: $pids"
kill -TERM $pids 2>/dev/null || true

elapsed=0
while [ "$elapsed" -lt "$TIMEOUT_SECONDS" ]; do
    if ! pgrep -x auto-aim > /dev/null 2>&1; then
        log "auto-aim exited after SIGTERM"
        exit 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
done

remaining_pids=$(pgrep -x auto-aim || true)
if [ -n "$remaining_pids" ]; then
    log "auto-aim did not exit in time, sending SIGKILL: $remaining_pids"
    kill -KILL $remaining_pids 2>/dev/null || true
fi

exit 0
