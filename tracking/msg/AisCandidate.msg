time stamp
# Maritime Mobile Service Identity (unique user ID)
uint32 mmsi
# Speed of the vessel over ground
float32 speed_over_ground
# Position accuracy 1=High(error<=10m) 0=Baja(error>10m)
uint8 position_accuracy
# Longitude +-180degrees East=Positive West=Negative
float32 longitude
# Latitude +-90degrees North=Positive South=Negative
float32 latitude
# Course over ground in degrees
float32 course_over_ground
# Bow orientation in degrees
uint16 true_heading

# Overall dimension/reference for position. See https://www.navcen.uscg.gov/?pageName=AISMessagesAStatic
uint16 dimension_a # 0-511m 511=511m or greater
uint16 dimension_b # 0-511m 511=511m or greater
uint16 dimension_c # 0-63m 63=63m or greater
uint16 dimension_d # 0-63m 63=63m or greater