print "Start simulator (SITL)"
import dronekit_sitl
import sys
import time


#Set up option parsing to get connection string
import argparse  

# Import DroneKit-Python
from dronekit import connect, VehicleMode

# [(43.47917960688476, -80.52704943477717), (43.47875045344238, -80.52704943477717), (43.4783213, -80.52704943477717), (43.477892146557615, -80.52704943477717), (43.47746299311523, -80.52704943477717), (43.47746299311523, -80.52662028133479), (43.477892146557615, -80.52662028133479), (43.4783213, -80.52662028133479), (43.47875045344238, -80.52662028133479), (43.47917960688476, -80.52662028133479), (43.47917960688476, -80.5261911278924), (43.47875045344238, -80.5261911278924), (43.4783213, -80.5261911278924), (43.477892146557615, -80.5261911278924), (43.47746299311523, -80.5261911278924), (43.47746299311523, -80.52576197445002), (43.477892146557615, -80.52576197445002), (43.4783213, -80.52576197445002), (43.47875045344238, -80.52576197445002), (43.47917960688476, -80.52576197445002), (43.47917960688476, -80.52533282100764), (43.47875045344238, -80.52533282100764), (43.4783213, -80.52533282100764), (43.477892146557615, -80.52533282100764), (43.47746299311523, -80.52533282100764)]


"""
CALL BACKS
"""

#Callback to print the location in global frames. 'value' is the updated value
def location_callback(self, attr_name, value):
 print "Location (Global): ", value

# Demonstrate getting callback on any attribute change
def wildcard_callback(self, attr_name, value):
    print " CALLBACK: (%s): %s" % (attr_name,value)


"""
FUNCTIONS
"""

def get_connection():
    parser = argparse.ArgumentParser(description='Generates max, min and current interval between message sent and ack recieved. Will start and connect to SITL if no connection string specified.')
    parser.add_argument('--connect', 
                       help="vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string=args.connect

    return connection_string

def readmission(aFileName):
    """
    Load a mission from a file into a list.

    This function is used by upload_mission().
    """
    print "Reading mission from file: %s\n" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    #Read mission from file
    missionlist = readmission(aFileName)

    print "\nUpload mission from a file: %s" % import_mission_filename
    #Clear existing mission from vehicle
    print ' Clear mission'
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print ' Upload mission'
    vehicle.commands.upload()


def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    missionlist = download_mission()
    output='QGC WPL 110\n'
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open("mission/"+aFileName, 'w') as file_:
        file_.write(output)


"""
CONTROLS
"""

def arm_and_takeoff(aTargetAltitude, vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"

    if v.mode.name == "INITIALISING":
        print "Waiting for vehicle to initialise"
        time.sleep(1)
    while vehicle.gps_0.fix_type < 2:
        print "Waiting for GPS...:", vehicle.gps_0.fix_type
        time.sleep(1)
    # Don't try to arm until autopilot is ready
    # vehicle has booted, EKF is ready, and the vehicle has GPS lock
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)





"""
ATTRIBUBTE DISPLAY
"""
def get_attributes(vehicle):
    # Get some vehicle attributes (state)
    print "Get some vehicle attribute values:"
    print " GPS: %s" % vehicle.gps_0
    print " Battery: %s" % vehicle.battery
    print " Last Heartbeat: %s" % vehicle.last_heartbeat
    print " Is Armable?: %s" % vehicle.is_armable
    print " System status: %s" % vehicle.system_status.state
    print " Mode: %s" % vehicle.mode.name    # settable

    print "Autopilot Firmware version: %s" % vehicle.version
    # print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
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


    # Get Vehicle Home location - will be `None` until first set by autopilot
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print " Waiting for home location ..."

    # We have a home location.
    print "\n Home location: %s" % vehicle.home_location



# Add a callback `location_callback` for the `global_frame` attribute.
# vehicle.add_attribute_listener('location.global_frame', location_callback)

# Wait 2s so callback can be notified before the observer is removed
# time.sleep(2)

# Remove observer - specifying the attribute and previously registered callback function
# vehicle.remove_message_listener('location.global_frame', location_callback)

# print "\nAdd attribute callback detecting any attribute change"
# vehicle.add_attribute_listener('*', wildcard_callback)


# print " Wait 1s so callback invoked before observer removed"
# time.sleep(1)

# print " Remove Vehicle attribute observer"
# Remove observer added with `add_attribute_listener()`
# vehicle.remove_attribute_listener('*', wildcard_callback)




def main():

    connection_string = get_connection()

    #Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle.
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)

    get_attributes(vehicle)

    # print "\n TAKE OFF" 
    # arm_and_takeoff(20, vehicle)

    upload_mission("mission/hickory_mission.txt")
    


    # Close vehicle object before exiting script
    vehicle.close()


    # Shut down simulator
    if not connection_string:
        sitl.stop()
    print("Completed")


if __name__ == "__main__":
    main()
