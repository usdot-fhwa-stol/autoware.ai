# This represents lanes that are next to each other.
# This relation object is only made if the lane change is available. 

# BlinkerType
uint8 NONE = 0
uint8 LEFT = 1
uint8 RIGHT = 2

# Id of refering lane
int32 lane_id

# Id of lane that is next to the refering lane. 
int32 next_lane_id

# Represents blinker that must be lit when doing this lane change.
# value of "blinker" must be one of "Blinker Type"
# e.g. if next_lane_id is left of lane_id, then blinker = LEFT
int32 blinker
