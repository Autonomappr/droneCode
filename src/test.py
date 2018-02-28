print "Start simulator (SITL)"
import dronekit_sitl

# [(43.47917960688476, -80.52704943477717), (43.47875045344238, -80.52704943477717), (43.4783213, -80.52704943477717), (43.477892146557615, -80.52704943477717), (43.47746299311523, -80.52704943477717), (43.47746299311523, -80.52662028133479), (43.477892146557615, -80.52662028133479), (43.4783213, -80.52662028133479), (43.47875045344238, -80.52662028133479), (43.47917960688476, -80.52662028133479), (43.47917960688476, -80.5261911278924), (43.47875045344238, -80.5261911278924), (43.4783213, -80.5261911278924), (43.477892146557615, -80.5261911278924), (43.47746299311523, -80.5261911278924), (43.47746299311523, -80.52576197445002), (43.477892146557615, -80.52576197445002), (43.4783213, -80.52576197445002), (43.47875045344238, -80.52576197445002), (43.47917960688476, -80.52576197445002), (43.47917960688476, -80.52533282100764), (43.47875045344238, -80.52533282100764), (43.4783213, -80.52533282100764), (43.477892146557615, -80.52533282100764), (43.47746299311523, -80.52533282100764)]



#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Generates max, min and current interval between message sent and ack recieved. Will start and connect to SITL if no connection string specified.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string=args.connect

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Import DroneKit-Python
from dronekit import connect, VehicleMode

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# Get some vehicle attributes (state)
print "Get some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
if not connection_string:
    sitl.stop()
print("Completed")

print "Autopilot Firmware version: %s" % vehicle.version
print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
print "Global Location: %s" % vehicle.location.global_frame
print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print "Local Location: %s" % vehicle.location.local_frame    #NED
print "Attitude: %s" % vehicle.attitude
print "Velocity: %s" % vehicle.velocity
print "GPS: %s" % vehicle.gps_0
print "Groundspeed: %s" % vehicle.groundspeed
print "Airspeed: %s" % vehicle.airspeed
print "Gimbal status: %s" % vehicle.gimbal
print "Battery: %s" % vehicle.battery
print "EKF OK?: %s" % vehicle.ekf_ok
print "Last Heartbeat: %s" % vehicle.last_heartbeat
print "Rangefinder: %s" % vehicle.rangefinder
print "Rangefinder distance: %s" % vehicle.rangefinder.distance
print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
print "Heading: %s" % vehicle.heading
print "Is Armable?: %s" % vehicle.is_armable
print "System status: %s" % vehicle.system_status.state
print "Mode: %s" % vehicle.mode.name    # settable
print "Armed: %s" % vehicle.armed    # settable