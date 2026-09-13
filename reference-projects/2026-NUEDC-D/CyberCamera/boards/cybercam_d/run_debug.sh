#!/bin/sh
set -eu
sudo systemctl stop cybercam-desktop.service
exec python3 -u main.py --backend csi --display builtin --display-fps 10 "$@"
