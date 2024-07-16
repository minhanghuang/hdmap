std_msgs/Header header

# Map type enumeration
uint8 MAP_TYPE_UNKNOWN   = 0
uint8 MAP_TYPE_OPENDRIVE = 1
uint8 MAP_TYPE_LANELET2  = 2
uint8 MAP_TYPE_OSM       = 3

string uuid

# Absolute path to the map file
string file_path

# Type of the map
uint8 map_type

# Map resolution in meters per pixel
float64 resolution
