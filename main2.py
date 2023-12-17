import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time
from geopy.distance import geodesic
from pymavlink import mavutil
import threading

from pynput import keyboard
from pynput.keyboard import Key, Listener

# Connect
connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string, wait_ready=True)


# The function of launching the drone and rising to the required height
def arm_and_takeoff(my_target_altitude):
    print("Starting the take-off function")

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.armed = True
    vehicle.channels.overrides = {'3': 2000}

    while not vehicle.armed:
        time.sleep(1)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        # print("Altitude: " + str(current_altitude))
        # Stop when the desired height is reached
        if my_target_altitude + 0.09 >= current_altitude >= my_target_altitude - 0.09:
            vehicle.channels.overrides = {'3': 1500}
            print("Completion of the take-off function")
            break
        # Decrease in speed at the end of the climb
        elif my_target_altitude - current_altitude < 8:
            vehicle.channels.overrides = {'3': 1650}
            time.sleep(0.005)
            vehicle.channels.overrides = {'3': 1500}
            time.sleep(0.005)


# Function for calculating the distance between points
def get_distance_metres(point1, point2):
    coords_1 = (point1.lat, point1.lon)
    coords_2 = (point2.lat, point2.lon)
    return geodesic(coords_1, coords_2).meters


# The function of calculating the distance to the end point
def distance_to_point_function(point):
    while True:
        distance_to_point = get_distance_metres(vehicle.location.global_relative_frame, point)
        print("Distance to point: " + str(round(distance_to_point, 2)))
        if round(distance_to_point) < 1:
            break
        time.sleep(0.5)


# Function for moving from point A to point B
def go_to_point(point_A, point_B):
    min_distance = get_distance_metres(point_A, point_B)

    while True:
        current_location = vehicle.location.global_relative_frame
        distance = get_distance_metres(current_location, point_B)
        min_distance = min(min_distance, distance)

        delta_lon = point_B.lon - current_location.lon
        delta_lat = point_B.lat - current_location.lat
        azimuth = math.atan2(delta_lon, delta_lat)

        # The copter stops
        if round(vehicle.location.global_relative_frame.lon, 5) == round(point_B_location.lon, 5) \
                and round(vehicle.location.global_relative_frame.lat, 5) == round(point_B_location.lat, 5):
            print("The copter is at point B")
            vehicle.channels.overrides = {'1': 1500, '2': 1500, '3': 1500, '4': 1500}
            break
        # The copter flies forward ((min_distance/40) sec)
        elif round(math.degrees(vehicle.attitude.yaw)) == round(math.degrees(azimuth)):
            vehicle.channels.overrides = {'2': 1200, '3': 1500}
            time.sleep(min_distance / 40)
        # Azimuth correction
        elif round(math.degrees(vehicle.attitude.yaw)) <= round(math.degrees(azimuth)):
            vehicle.channels.overrides = {'2': 1500, '3': 1500, '4': 1550}
            time.sleep(0.01)
        # Azimuth correction
        elif round(math.degrees(vehicle.attitude.yaw)) >= round(math.degrees(azimuth)):
            vehicle.channels.overrides = {'2': 1500, '3': 1500, '4': 1450}
            time.sleep(0.01)


# Function for rotation
def turn_function(angle):
    if angle >= 180:
        angle = angle - 360
    while True:
        # Stop when the desired angle is reached
        if round(math.degrees(vehicle.attitude.yaw)) == angle:
            vehicle.channels.overrides = {'1': 1500, '2': 1500, '3': 1500, '4': 1500}
            break

        vehicle.channels.overrides = {'2': 1500, '3': 1500, '4': 1550}
        time.sleep(0.01)
        vehicle.channels.overrides = {'2': 1500, '3': 1500, '4': 1500}
        time.sleep(0.01)


# A function for implementing a joystick
def on_press(key):
    try:
        if key == keyboard.Key.esc:
            return False

        elif key == keyboard.Key.space:
            vehicle.channels.overrides = {'1': 1500, '2': 1500, '3': 1500, '4': 1500}
            vehicle.flush()
            print("SPASE")

        elif key.char == 'w':
            vehicle.channels.overrides = {'2': 1200, '3': 1500}
            vehicle.flush()
            print("W")

        elif key.char == 's':
            vehicle.channels.overrides = {'2': 1800, '3': 1500}
            vehicle.flush()
            print("S")

        elif key.char == 'a':
            vehicle.channels.overrides = {'4': 1450, '3': 1500}
            vehicle.flush()
            print("A")

        elif key.char == 'd':
            vehicle.channels.overrides = {'4': 1550, '3': 1500}
            vehicle.flush()
            print("D")

        elif key.char == '+':
            vehicle.channels.overrides = {'3': 1650}
            vehicle.flush()
            print("+")

        elif key.char == '-':
            vehicle.channels.overrides = {'3': 1350}
            vehicle.flush()
            print("-")

    except AttributeError:
        pass


if __name__ == "__main__":
    try:
        target_altitude = 100  # height
        point_A_location = LocationGlobalRelative(50.450739, 30.461242)  # point A
        point_B_location = LocationGlobalRelative(50.443326, 30.448078)  # point B
        yaw = 350  # yaw

        # Take off
        arm_and_takeoff(target_altitude)

        # Distance to point
        t = threading.Thread(target=distance_to_point_function, args=(point_B_location,))
        t.daemon = True
        t.start()

        # Go to point
        go_to_point(point_A_location, point_B_location)

        # Yaw
        turn_function(yaw)

        print("""
        Press "W" to move forward
        Press "S" to move back
        Press "A" to turn left
        Press "D" to turn right
        Press "+" to increase height
        Press "-" to decrease height
        Press "SPASE" to stabilize
        Press "Esc" to exit
        """)
        # Joystick call
        with Listener(on_press=on_press) as listener:
            listener.join()

    except KeyboardInterrupt:
        pass
    finally:
        vehicle.close()
