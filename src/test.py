print "Start simulator (SITL)"
import dronekit_sitl
import sys
import time
import os
import math
from pymavlink import mavutil


import argparse  
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative, LocationGlobal
# from pudb import set_trace; set_trace()

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
    sitl = None

    return connection_string, sitl

def readmission(aFileName, vehicle):
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

def upload_mission(aFileName, vehicle):
    """
    Upload a mission from a file.
    """
    #Read mission from file
    missionlist = readmission(aFileName, vehicle)

    print "\nUpload mission from a file: %s" % aFileName
    #Clear existing mission from vehicle
    print ' Clear mission'
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print ' Upload mission'
    vehicle.commands.upload()

def download_mission(vehicle):
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

def save_mission(aFileName, vehicle):
    """
    Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    missionlist = download_mission(vehicle)
    output='QGC WPL 110\n'
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open("mission/"+aFileName, 'w') as file_:
        file_.write(output)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint(vehicle):
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

"""
CONTROLS
"""

def arm_and_takeoff(aTargetAltitude, vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"

    if vehicle.mode.name == "INITIALISING":
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
    # Get all vehicle attributes (state)
    print "\nGet all vehicle attribute values:"
    print " Autopilot Firmware version: %s" % vehicle.version
    print "   Major version number: %s" % vehicle.version.major
    print "   Minor version number: %s" % vehicle.version.minor
    print "   Patch version number: %s" % vehicle.version.patch
    print "   Release type: %s" % vehicle.version.release_type()
    print "   Release version: %s" % vehicle.version.release_version()
    print "   Stable release?: %s" % vehicle.version.is_stable()
    print " Autopilot capabilities"
    print "   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float
    print "   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float
    print "   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int
    print "   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int
    print "   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union
    print "   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp
    print "   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target
    print "   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned
    print "   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int
    print "   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain
    print "   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target
    print "   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination
    print "   Supports mission_float message type: %s" % vehicle.capabilities.mission_float
    print "   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration
    print " Global Location: %s" % vehicle.location.global_frame
    print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
    print " Local Location: %s" % vehicle.location.local_frame
    print " Attitude: %s" % vehicle.attitude
    print " Velocity: %s" % vehicle.velocity
    print " GPS: %s" % vehicle.gps_0
    print " Gimbal status: %s" % vehicle.gimbal
    print " Battery: %s" % vehicle.battery
    print " EKF OK?: %s" % vehicle.ekf_ok
    print " Last Heartbeat: %s" % vehicle.last_heartbeat
    print " Rangefinder: %s" % vehicle.rangefinder
    print " Rangefinder distance: %s" % vehicle.rangefinder.distance
    print " Rangefinder voltage: %s" % vehicle.rangefinder.voltage
    print " Heading: %s" % vehicle.heading
    print " Is Armable?: %s" % vehicle.is_armable
    print " System status: %s" % vehicle.system_status.state
    print " Groundspeed: %s" % vehicle.groundspeed    # settable
    print " Airspeed: %s" % vehicle.airspeed    # settable
    print " Mode: %s" % vehicle.mode.name    # settable
    print " Armed: %s" % vehicle.armed    # settable

    # Get Vehicle Home location - will be `None` until first set by autopilot
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print " Waiting for home location ..."

    # We have a home location.
    print "\n Home location: %s" % vehicle.home_location


    
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

    connection_string, sitl = get_connection()

    #Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle.
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)
    vehicle.wait_ready('autopilot_version')

    get_attributes(vehicle)

    # Add a callback `location_callback` for the `global_frame` attribute.
    vehicle.add_attribute_listener('location.global_frame', location_callback)

    print "\nSet Vehicle.mode = GUIDED (currently: %s)" % vehicle.mode.name 
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
        print " Waiting for mode change ..."
        time.sleep(1)


    upload_mission("missions/hickory_mission.txt", vehicle)
    arm_and_takeoff(20, vehicle)

    print "Starting mission"
    # Reset mission set to first (0) waypoint
    vehicle.commands.next=0

    print "Set default/target groundspeed to 1"
    vehicle.groundspeed = 20
    vehicle.mode = VehicleMode("AUTO")

    # os.system("path_planning.py 1 8 Hickory St West Waterloo Ontario Canada")


    # #Create a message listener using the decorator.
    # @vehicle.on_message('RANGEFINDER')
    # def listener(self, name, message):
    #     print message

    # Monitor mission. 
    # Demonstrates getting and setting the command number 
    # Uses distance_to_current_waypoint(), a convenience function for finding the 
    #   distance to the next waypoint.

    while True:
        nextwaypoint=vehicle.commands.next
        print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint(vehicle))
      
        # if nextwaypoint==3: #Skip to next waypoint
        #     print 'Skipping to Waypoint 5 when reach waypoint 3'
        #     vehicle.commands.next = 5
        if nextwaypoint==25: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print "Exit 'standard' mission when start heading to final waypoint (25)"
            break;
        time.sleep(1)

    print 'Return to launch'
    vehicle.mode = VehicleMode("RTL")


    #Close vehicle object before exiting script
    print "Close vehicle object"
    vehicle.close()



    # # Shut down simulator
    if not sitl == None:
        sitl.stop()
    print("Completed")


if __name__ == "__main__":
    main()
