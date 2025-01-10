from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, math, argparse
import matplotlib.pyplot as plt


def calculate_bearing(lat1, lon1, lat2, lon2):
    delta_lon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    bearing = math.atan2(x, y)
    return math.degrees(bearing) % 360


def get_perpendicular_location(original_location, bearing, distance, alt):
    earth_radius = 6378137.0
    perp_bearing = (bearing + 90) % 360

    d_lat = (distance * math.cos(math.radians(perp_bearing))) / earth_radius
    d_lon = (distance * math.sin(math.radians(perp_bearing))) / (
        earth_radius * math.cos(math.radians(original_location.lat))
    )

    new_lat = original_location.lat + math.degrees(d_lat)
    new_lon = original_location.lon + math.degrees(d_lon)

    return LocationGlobal(new_lat, new_lon, original_location.alt + alt)


def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                          mavMode,
                                          0, 0, 0, 0, 0, 0)


def get_location_offset_meters(original_location, dNorth, dEast, alt):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt + alt)


def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def plot_mission_path(waypoints, actual_path):
    plt.figure(figsize=(10, 6))
    plt.plot([wp.lon for wp in waypoints], [wp.lat for wp in waypoints], 'bo-', label='Planned Path')
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Mission Path")
    plt.legend()
    plt.grid()
    plt.show()


connection_string = '127.0.0.1:14540'
MAV_MODE_AUTO = 4

parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect

print("Connecting")
vehicle = connect(connection_string, wait_ready=True)

home_position_set = False


@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True


while not home_position_set:
    print("Waiting for home position...")
    time.sleep(1)

print(f"Type: {vehicle._vehicle_type}")
print(f"Armed: {vehicle.armed}")
print(f"System status: {vehicle.system_status.state}")
print(f"GPS: {vehicle.gps_0}")
print(f"Alt: {vehicle.location.global_relative_frame.alt}")

PX4setMode(MAV_MODE_AUTO)
time.sleep(1)

cmds = vehicle.commands
cmds.clear()

home = vehicle.location.global_relative_frame
waypoints = []
for i in range(15):
    dNorth = i * 10
    dEast = 0 if i % 2 == 0 else 10
    wp = get_location_offset_meters(home, dNorth, dEast, 10)
    waypoints.append(wp)

bearing = calculate_bearing(
    waypoints[9].lat, waypoints[9].lon, waypoints[10].lat, waypoints[10].lon
)
perpendicular_wp = get_perpendicular_location(waypoints[10], bearing, 100, 10)
waypoints.insert(10, perpendicular_wp)

for idx, wp in enumerate(waypoints):
    cmd_type = mavutil.mavlink.MAV_CMD_NAV_LAND if idx == len(waypoints) - 1 else mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                  cmd_type, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

cmds.upload()
time.sleep(2)

vehicle.armed = True
vehicle.commands.next = 0  
start_time = time.time()

actual_path = []
nextwaypoint = vehicle.commands.next
total_distance = 0
for i in range(len(waypoints)-1):
    total_distance += calculate_distance(waypoints[i].lat,waypoints[i].lon, waypoints[i+1].lat, waypoints[i+1].lon)
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
       
        nextwaypoint = vehicle.commands.next
        print(f"Moving to waypoint {nextwaypoint}")
    
    current_location = vehicle.location.global_relative_frame
  
    next_wp = waypoints[vehicle.commands.next - 1] 
    distance_to_next = calculate_distance(
        current_location.lat, current_location.lon,
        next_wp.lat, next_wp.lon
    )

    remaining_distance = distance_to_next
    for i in range(vehicle.commands.next, len(waypoints)):
        wp1 = waypoints[i - 1] if i > 0 else current_location
        wp2 = waypoints[i]
        remaining_distance += calculate_distance(wp1.lat, wp1.lon, wp2.lat, wp2.lon)
    
    ground_speed = vehicle.groundspeed  
    if ground_speed > 0:
        estimated_time = remaining_distance / ground_speed
    else:
        estimated_time = float('inf')  
    print(f"Remaining distance to mission completion: {remaining_distance:.2f} meters")
    print(f"Estimated time to mission completion: {estimated_time:.2f} seconds")

while vehicle.armed:
    print("Landing...")
    actual_path.append(vehicle.location.global_relative_frame)
    time.sleep(1)

print("Mission complete. Drone landed.")
plot_mission_path(waypoints, actual_path)
vehicle.close()
time.sleep(1)
