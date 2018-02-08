# Reset Sim Parameters
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
exit

sim_vehicle.py --console --map --aircraft test
wp load ../Tools/autotest/ArduPlane-Missions/CMAC-toff-loop.txt
arm throttle
mode auto
