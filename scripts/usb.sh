#!/usr/bin/env bash

# testing the GPU box

export XDG_CACHE_HOME=/data/tinycache
mkdir -p $XDG_CACHE_HOME

if [ -d /data/openpilot/tinygrad_repo/examples ]; then
  cd /data/openpilot/tinygrad_repo/examples
elif [ -d /data/openpilot/examples ]; then
  cd /data/openpilot/examples
else
  echo "Unable to locate tinygrad examples directory"
  exit 1
fi
while true; do
  AMD=1 AMD_IFACE=usb python ./beautiful_cartpole.py
  sleep 1
done
