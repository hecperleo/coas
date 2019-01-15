# Radar Extended Track Report msg

##### CONSTANTS #####
uint8 UDPMSGSIZE=188
uint8 GAINXPOS=152
uint8 GAINYPOS=156
uint8 GAINVXPOS=160
uint8 GAINVYPOS=164
uint8 INNOERRXPOS=168
uint8 INNOERRYPOS=172

##### FIELDS #####
# Normal Message fields
RadarNormalTrackReport normal_msg 

# Estimation gain on X coordinate
float32 gain_x

# Estimation gain on Y coordinate
float32 gain_y

# Estimation gain on X coordinate
float32 gain_vx

# Estimation gain on Y coordinate
float32 gain_vy

# Kalman innovation element (0,0)
float32 innovation_error_x

# Kalman innovation element (1,1)
float32 innovation_error_y