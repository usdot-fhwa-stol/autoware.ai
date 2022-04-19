# Message describing the contents of a lanelet2 routing graph such that it can be reconstructed by the recieving component


# A list of lanelets representing vertices in the graph and their associated edges
# This structure mirrors an adjacency list representation of a directed graph 
RoutingGraphVertexAndEdges[] lanelet_vertices

# A list of areass representing vertices in the graph and their associated edges
RoutingGraphVertexAndEdges[] area_vertices

# The lanelet2 participant type used to generate this graph
# The lanelets and areas in this graph will only be present if they are passable by this participant type
string participant_type

# The maximum number of routing cost ids used to compute edge costs in this graph
# This field is an optimization to prevent the need to count the number of unique routing cost ids in the edges
int64 num_unique_routing_cost_ids

