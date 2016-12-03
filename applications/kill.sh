#./bin/sh

killall mixer
killall depthpidpy
killall yawpidpy
killall pitchpidpy
killall rollpidpy
killall pulsepidpy

#pkill -f ".*pitchpidpy"
#pkill -f ".*yawpidpy"
#pkill -f ".*rollpidpy"
#pkill -f ".*depthpidpy"
#pkill -f ".*pulsepidpy"
#pkill -f ".*mixer"

./bin/zerothrusters

