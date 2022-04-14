# This message is a description of a single vertex (lanelet or area) and its edges
# in a Lanelet2 Routing Graph. The edges are stored as a collection of equally sized areas where each index corresponds to one edge
# The reason edges are stored as multiple array fields instead of independent message objects
# is because ROS2 DDS implementations have known performance issues with heavily nested data structures. 

int64 lanelet_or_area # Lanelet of interest (the vertex)

# The following 4 arrays must have the same size and ordering such that each index corresponds to the same routing graph edge 
int64[] lanelet_or_area_ids # The other vertices connected to the vertex of interest
float64[] edge_routing_costs # The routing costs of the edges
uint16[] edge_routing_cost_source_ids # The ids of the routing cost generators used to compute the routing cost of each edge
uint8[] edge_relations # The relation type of the edge

# Enum of edge relation types
# See the enum class RelationType in Lanelet2 for relation descriptions
uint8 RELATION_NONE=0
uint8 RELATION_SUCCESSOR=1
uint8 RELATION_LEFT=2
uint8 RELATION_RIGHT=3
uint8 RELATION_ADJACENT_LEFT=4
uint8 RELATION_ADJACENT_RIGHT=5
uint8 RELATION_CONFLICTING=6
uint8 RELATION_AREA=7