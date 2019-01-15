# Radar Basic Track Report msg

##### CONSTANTS #####
uint8 UDPMSGSIZE=80 # 76
uint8 TRACKIDPOS=4
uint8 NHITSPOS=8
uint8 ESTATUSPOS=10
uint8 NRUNSPOS=12
uint8 ESTXPOS=16
uint8 ESTYPOS=20
uint8 ESTAZIPOS=24
uint8 ESTRANGEPOS=28
uint8 ESTSPEEDPOS=32
uint8 ESTCOURSEPOS=36
uint8 ESTLATPOS=40
uint8 ESTLONPOS=44
uint8 TIMESTAMPLSBPOS=48
uint8 MEARANGESIZEPOS=56
uint8 MEAAZISIZEPOS=60

##### FIELDS #####
uint32 msg_identifier

uint32 track_id

# Track status
uint8 e_status

# Track detections counter
uint16 n_hits 

# Antenna scan counter. Number of antenna scans from track creation
uint16 n_runs

# Estimated x distance from radar (meters)
float32 estimated_x

# Estimated y distance from radar (meters)
float32 estimated_y

# Estimated azimuth relative to radar [0-360] degrees
float32 estimated_azi

# Estimated range to radars (meters)
float32 estimated_range

# Estimated absolute speed (meters per second)
float32 estimated_speed

# Estimated absolute course (meters)
float32 estimated_course

# Estimated latitude (degrees)
float32 estimated_lat

# Estimated longitude (degrees)
float32 estimated_lon

# Detection time in ms
uint64 time_stamp

# Measured size in slant range
float32 measured_range_size

# Measured size in azimuth
float32 measured_azimuth_size
