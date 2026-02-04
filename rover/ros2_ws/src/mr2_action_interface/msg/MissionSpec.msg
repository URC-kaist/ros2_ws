# Mission specification for Mission Master
# mission_type: 0=UNKNOWN, 1=GNSS_ONLY, 2=COVER_VISION
# detection_method: 0=NONE, 1=ARUCO, 2=YOLO
# object_type (used by YOLO): 0=MALLET, 1=PICK, 2=BOTTLE
uint32 mission_id
uint8 mission_type
uint8 detection_method
int32 object_type

# WGS84 target (required for GNSS_ONLY and COVER_VISION)
float64 target_latitude
float64 target_longitude

# Coverage parameters (used by COVER_VISION)
float64 target_radius
uint32 waypoint_count
