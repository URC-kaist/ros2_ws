# Command channel for Mission Master
# command: 0=NOOP, 1=PAUSE, 2=RESUME, 3=ABORT
uint8 command

# Optional: clear costmaps when pausing or aborting
bool clear_costmap

# Optional: target mission id (0 = current mission)
uint32 mission_id
