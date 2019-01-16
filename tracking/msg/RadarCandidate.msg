# Radar Candidate Msg

uint8 BASICREPORTLEVEL=0
uint8 NORMALREPORTLEVEL=1

# Track report level
# Report levels are incremental, fields of a lower level report are also included in
# a higher level one
uint8 report_level

# Time at which the message is generated after receiving data from radar
time stamp

##############################################
# Fields populated in case of BASICREPORTLEVEL
# Track unique identifier
uint32 track_id
# Track status
uint8 e_status
# Track detections counter
uint16 n_hits
# Estimated x distance from radar
float32 estimated_x
# Estimated y distance from radar
float32 estimated_y
# Estimated azimuth relative to radar [0-360]
float32 estimated_azimuth
# Estimated range to radar
float32 estimated_range
# Estimated absolute speed (meters per second)
float32 estimated_speed
# Estimated absolute course (degrees)
float32 estimated_course
# Estimated latitude (degrees)
float32 estimated_latitude
# Estimated longitude (degrees)
float32 estimated_longitude
# Detection time in ms from Epoch
uint64 time_stamp
# Measured size in slant range
float32 measured_range_size
# Meaused size in azimuth
float32 measured_azimuth_size
##############################################

# Fields also populated in case of NORMALREPORTLEVEL
# Measured x cartesian position
float32 measured_x
# Measured y cartesian position
float32 measured_y
# Measured azimuth
float32 measured_azimuth
# Measured radar range
float32 measured_range
# Measured speed
float32 measured_speed
# Measured course
float32 measured_course
# Indication of track quality
float32 score