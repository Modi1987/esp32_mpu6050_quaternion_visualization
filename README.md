
# MPU6050 Cube Animation Visualization

This is a Python3-based visualization tool that animates a rotating cube, reflecting the real-time orientation of an MPU6050 sensor connected to an ESP32 microcontroller.
Demo

Below is a GIF that shows the output you'll see as you move the MPU6050 sensor:

**`![Cube Animation](./data/animation.gif)`**

## Requirements

- ESP32 with an MPU6050 sensor attached
- Python 3.x
- Required Python libraries (install via requirements.txt):

```bash
pip install -r requirements.txt
```

## Hardware Setup

- Flash the ESP32 with the code provided in the arduino folder.
- Connect the MPU6050 to the ESP32 as per the wiring instructions in the comments of the Arduino source code.

## Software Setup

- Connect your PC to the WiFi network broadcasted by the ESP32.
- On your PC, run the Python visualization tool:

```bash
python3 ./python/cube_rotation/main.py
```

This will open a window showing the animated cube that updates based on the real-time orientation of the MPU6050 sensor.


