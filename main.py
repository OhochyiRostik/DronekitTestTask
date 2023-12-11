from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from geopy.distance import geodesic
from pymavlink import mavutil

# Connect
connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string, wait_ready=True)


# The function of launching the drone and rising to the required height
def arm_and_takeoff(my_target_altitude):
    print("Starting the take-off function")
    while not vehicle.is_armable:
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(my_target_altitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= my_target_altitude * 0.99:
            print("Completion of the take-off function")
            break
        time.sleep(1)


# Function for calculating the distance to a point
def get_distance_metres(point1, point2):
    coords_1 = (point1.lat, point1.lon)
    coords_2 = (point2.lat, point2.lon)
    return geodesic(coords_1, coords_2).meters


# Move to point function
def goto_position(target_location, speed=5):
    print("The movement to the point is started")
    vehicle.simple_goto(target_location, groundspeed=speed)
    while True:
        distance = get_distance_metres(vehicle.location.global_frame, target_location)
        print("Distance to point:" + str(distance))
        if distance <= 1:
            print("The movement to the point is complete")
            break
        time.sleep(1)


# Rotation function
def condition_yaw(heading, relative=False):
    print("Rotation has started")
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    print("The rotation is complete")


# Landing function
def landing():
    print("Landing has begun")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode.name != "LAND":
        print("Landing")
        time.sleep(1)
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude <= 1 * 0.01:
            print("Landing is over")
            break
        time.sleep(1)


if __name__ == "__main__":
    try:
        # Take off
        target_altitude = 100  # height
        arm_and_takeoff(target_altitude)

        # Movement to the point (If the starting position is not at point A, then you need to fly to point A first)
        flight_speed = 15  # flight speed
        # Point A
        point_A = LocationGlobalRelative(50.450739, 30.461242, 100)
        if vehicle.location.global_frame != point_A:
            goto_position(point_A)

        # Point B
        point_B = LocationGlobalRelative(50.443326, 30.448078, 100)
        # point_B = LocationGlobalRelative(50.449974, 30.460931, 10)
        goto_position(point_B, flight_speed)

        # Rotation
        value_heading = 350  # yaw
        condition_yaw(value_heading, relative=True)
        time.sleep(5)

        # Landing
        landing()

    except KeyboardInterrupt:
        pass
    finally:
        vehicle.close()
