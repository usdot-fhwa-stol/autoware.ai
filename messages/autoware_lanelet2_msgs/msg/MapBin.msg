# This contains the lanelet map in binary format.

# Timestamp and frame_id of the map
std_msgs/Header header

# sequence ID: consecutively increasing ID
uint32 seq_id

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

# Description of the Routing Graph following this update.
# This field will only be populated if has_routing_graph == true
autoware_lanelet2_msgs/RoutingGraph routing_graph

# If true then this update contains a new routing_graph
# In standard usage this would only be set if invalidates_route==true or if this is the first map broadcast of a new map version
bool has_routing_graph
