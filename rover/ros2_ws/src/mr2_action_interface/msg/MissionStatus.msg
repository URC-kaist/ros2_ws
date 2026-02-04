# Mission Master status report
# state: 0=IDLE, 1=RUNNING, 2=PAUSED, 3=COMPLETED, 4=FAILED
builtin_interfaces/Time stamp
MissionSpec active_mission
uint8 state
bool arrival

# Coverage progress (optional)
uint32 current_waypoint_index
uint32 total_waypoints

# Nav2 feedback (meters). Use negative if unknown.
float32 distance_remaining

# Optional human-readable detail or error
string detail
