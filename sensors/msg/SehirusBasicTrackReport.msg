# Sehirus Basic Track Report msg

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
uint32 msgIdentifier

uint32 trackID

# Track status
uint8 eStatus

# Track detections counter
uint16 nHits 

# Antenna scan counter. Number of antenna scans from track creation
uint16 nRuns

# Estimated x distance from radar (meters)
float32 estX

# Estimated y distance from radar (meters)
float32 estY

# Estimated azimuth relative to radar [0-360] degrees
float32 estAzi

# Estimated range to radars (meters)
float32 estRange

# Estimated absolute speed (meters per second)
float32 estSpeed

# Estimated absolute course (meters)
float32 estCourse

# Estimated latitude (degrees)
float32 estLat

# Estimated longitude (degrees)
float32 estLon

# Detection time in ms
uint64 timeStamp

# Measured size in slant range
float32 meaRangeSize

# Measured size in azimuth
float32 meaAzimuthSize
