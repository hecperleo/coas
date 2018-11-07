#!/bin/bash
# This is a test script
# This script read the file heartbeat.bin which has the same layout as the sehirus heartbeat message
# Then pass the content to netcat to send it to UDP port 6500 in localhost

while true
do
cat heartbeat.bin | nc -u localhost 6500  
done