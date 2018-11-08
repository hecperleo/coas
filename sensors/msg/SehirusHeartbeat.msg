# Do not use tab to define messages

# Sehirus Heartbeat msg

##### Constants #####
uint8 UDPMSGSIZE=72 # UDP message size in bytes
uint8 SERVERNAMESIZE=16 # server name string length in bytes
uint8 SERVERNAMEPOS=4 # server name first byte position in UDP message
uint8 CMDLISTENPOS=23
uint8 CMDCONNECTEDPOS=22
uint8 CMDPORTPOS=20
uint8 RAWDISTTPUTPOS=24
uint8 PRODISTTPUTPOS=28
uint8 RAWRECTPUTPOS=32
uint8 SRCSTARTEDPOS=39
uint8 BUFFERFULLPOS=38
uint8 CPULOADPOS=36
uint8 CURRENTSRCPERIODPOS=42
uint8 NSCANSPOS=44
uint8 NAVDATAPRESENTPOS=56
uint8 LATITUDEPOS=48
uint8 LONGITUDEPOS=52

##### Fields #####
# Identifier ID constant 0x01AB0101
uint32 msgIdentifier

string serverName

# Status of the command listening
# port. 0=Not connected 1=Connected
uint8 commandListen

# Client connection.
# 0=Not connected 1=Connected
uint8 commandConnected

# Command listening port
uint16 commandPort

# Bandwith in MB/s
float32 rawDistributionThroughput
float32 proDistributionThroughput
float32 rawRecordingThroughput

# 0=Stop 1=Started
uint8 sourceStarted

# 0=Not full 1=Full
uint8 bufferFull

# Cpu Load 0-1000 0.1% precission
uint16 cpuLoad

# Antena period in ms
uint16 currentSourcePeriod

# currentSourcePRF

# number of antenna scans from last source start
uint32 nScans

# Navigation Data Present
# 0=No data
# 1= Data receiving set for all
# Position, speed over ground
# course, heading
uint8 navDataPresent

# Radar Latitude [-90.0-90.0] Degrees
float32 latitude

# Radar Longitude [-180.0-180.0] Degrees
float32 longitude