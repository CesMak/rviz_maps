Header header
float64 latitude
float64 longitude
float64 altitude
float64 scale
int8    status
float64 lat_center
float64 long_center

#Example:


#Notes Status of NavSatFix Message:
#int8 STATUS_NO_FIX =  -1        # unable to fix position            /Blue
#int8 STATUS_FIX =      0        # unaugmented fix                   /Blue
#int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation /Blue
#int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation    /Blue
#status=3 -> single solution  /Red
#status=4 -> float solution   /Yellow
#status=5 -> fix solution (AR is solved)  /Green
