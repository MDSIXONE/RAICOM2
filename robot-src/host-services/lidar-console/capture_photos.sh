#!/bin/sh
set -eu
exec python3 /home/pi/oumax-lidar-console/capture_photos.py "$@"
