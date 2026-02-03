# Mission specification for Mission Master
# mission_type: 0=UNKNOWN, 1=GNSS_ONLY, 2=COVER_VISION
# detection_method: 0=NONE, 1=YOLO, 2=ARUCO
uint32 mission_id
uint8 mission_type
uint8 detection_method

# WGS84 target (required for GNSS_ONLY and COVER_VISION)
float64 target_latitude
float64 target_longitude

# Coverage parameters (used by COVER_VISION)
float64 target_radius
uint32 waypoint_count
