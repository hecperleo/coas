# Sehirus Normal Track Report msg

##### CONSTANTS #####
uint8 UDPMSGSIZE=160 #152
uint8 MEAXPOS=76
uint8 MEAYPOS=80
uint8 MEAAZIPOS=84
uint8 MEARANGEPOS=88
uint8 MEASPEEDPOS=92
uint8 MEACOURSEPOS=96
uint8 PREXPOS=100
uint8 PREYPOS=104
uint8 PREAZIPOS=108
uint8 PRERANGEPOS=112
uint8 PRESPEEDPOS=116
uint8 PRECOURSEPOS=120
uint8 SCOREPOS=124
uint8 WEIGHTPOS=128
uint8 STRENGTHPOS=132
uint8 DENSITYPOS=136

##### FIELDS #####
# Basic Message fields
SehirusBasicTrackReport basicMsg

# Measured X cartesian position
float32 meaX

# Measured Y cartesian position
float32 meaY

# Measured azimuth
float32 meaAzi

# Measured radar range
float32 meaRange

# Measured speed
float32 meaSpeed

# Measured course
float32 meaCourse

# Predicted X cartesian position
float32 preX

# Predicted Y cartesian position
float32 preY

# Predicted radar range
float32 preRange

# Predicted speed
float32 preSpeed

# Predicted course
float32 preCourse

# Indication of track quality [0.0-1.0] 0=min 1=max
float32 score

# Number of valid samples
uint32 weight

# Sum of the level of the samples indicated by weight
uint32 strength

# Strength / weight
float32 density