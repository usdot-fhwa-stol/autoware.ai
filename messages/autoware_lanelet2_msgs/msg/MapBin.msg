# This contains the lanelet map in binary format.

# Timestamp and frame_id of the map
Header header

# version of map format. keep this as empty string if unnecessary
string format_version

# version of map version. monotonically increasing integer starting at 1
uint32 map_version

# binary data of lanelet2 map. This is meant to be filled using toBinMsg() in lanelet2_extension library
int8[] data

# The route filename
string route_id

# The version of current route, monotonically increasing integer starting at 1
uint32 route_version

# route flag to set the route invalidation status
bool invalidates_route
