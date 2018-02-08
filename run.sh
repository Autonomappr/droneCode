# Reset Sim Parameters
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w &
export pid=$!
sleep 10
kill -2  $pid

sim_vehicle.py --console --map --aircraft test
wp load ../Tools/autotest/ArduPlane-Missions/CMAC-toff-loop.txt
arm throttle
mode auto
