#!/bin/bash

tmux new-session -d -s gfm /home/gfmrpi/gfm_bin/open_challenge

# tmux new-session -d -s gfm /home/gfmrpi/gfm_bin/obstacle_challenge

# Infinite open_challenge
# tmux new-session -d -s gfm bash -c '
#     while true; do
#         if [ -f /tmp/stop_gfm ]; then
#             echo "Stop file detected — exiting loop."
#             rm -f /tmp/stop_gfm
#             break
#         fi
# 
#         /home/gfmrpi/gfm_bin/open_challenge
#         echo "open_challenge exited with code $? — restarting in 3 second..."
#         sleep 3
#     done
# '

# Infinite obstacle_challenge
# tmux new-session -d -s gfm bash -c '
#     while true; do
#         if [ -f /tmp/stop_gfm ]; then
#             echo "Stop file detected — exiting loop."
#             rm -f /tmp/stop_gfm
#             break
#         fi
# 
#         /home/gfmrpi/gfm_bin/obstacle_challenge
#         echo "obstacle_challenge exited with code $? — restarting in 3 second..."
#         sleep 3
#     done
# '

# tmux new-session -d -s gfm bash -c '
#     /home/gfmrpi/gfm_bin/scan_map_outer && sleep 5 && /home/gfmrpi/gfm_bin/scan_map_inner
#     tmux kill-session -t gfm
# '
