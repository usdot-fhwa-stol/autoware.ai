# This represents lane connections in a map.

# BlinkerType
uint8 NONE = 0
uint8 LEFT = 1
uint8 RIGHT = 2

# Id of refering Lane object.
int32 lane_id

# Id of Lane objects that follows after refering lane. 
int32 next_lane_id

# Blinker that must be lit when doing driving towards the next_lane.
# Must be one of "Blinker Type"
# e.g. if next_lane_id is left turning lane in intersection, then blinker = LEFT
int32 blinker
