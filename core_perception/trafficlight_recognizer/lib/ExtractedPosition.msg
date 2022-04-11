# This message defines the information required to describe a lamp (bulb)
# in a traffic signal, according to the information extracted from the ADAS Map
# and the extrinsic camera calibration

int32 signal_id  # Traffic Signal Lamp ID
int32 u         # Lamp ROI x in image coords
int32 v         # Lamp ROI y in image coords
int32 radius    # Lamp Radius
float64 x       # X position in map coordinates
float64 y       # Y position in map coordinates
float64 z       # Z position in map coordinates
float64 hang    # Azimuth "Horizontal Angle"
int8 type       # Lamp Type (red, yellow, green, ...)
int32 link_id    # Closest LinkID (lane) in VectorMap
int32 pl_id      # PoleID to which this Lamp belongs to
