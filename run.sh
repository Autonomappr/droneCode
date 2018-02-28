# Reset Sim Parameters
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w &
export pid=$!
sleep 100
kill -2  $pid

sim_vehicle.py --console --map --aircraft test -L Waterloo
sleep 20
wp load ../Tools/autotest/ArduPlane-Missions/CMAC-toff-loop.txt
arm throttle
mode auto
