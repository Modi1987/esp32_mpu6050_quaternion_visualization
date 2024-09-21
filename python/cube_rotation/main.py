from utils.cube_animator import quaternion2matrix, rotz, Cube
from utils.esp32_quat_over_wifi import Esp32
import time

# connect to ESP32 to get quaternion data
esp = Esp32()
if esp.start_streaming():  # Start streaming and check if ESP32 responds
    print("Connected succesfully to EPS32 and receiving quaternion data!")
else:
    raise Exception("Issue with connecting/acquiring data from IMU over WiFi!")

# Main code to animate cube
render_rate_hz = 100.0
cube = Cube(render_rate_hz)

try:
    while True:
        # Get the rotation quaternion and matrix
        measurements = esp.get_measurements()
        if measurements is None:
            continue
        [q, gyro] = measurements
        R = quaternion2matrix(q)
        # Render the cube with the current rotation matrix
        cube.render_cube(R)
except KeyboardInterrupt:
        print("Bye bye!")
        sys.exit()

