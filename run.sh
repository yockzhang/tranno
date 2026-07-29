#!/bin/bash
# Run WITH GUI via WSLg. Execute from WSL Ubuntu shell:  bash run.sh
docker run -it --rm --name tranno_sim \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v /mnt/d/tranno_robot/ws:/ws \
  tranno_sim bash
