import socket
import time
import struct
import numpy as np

class Esp32():
    def __init__(self):
        self.UDP_IP = "192.168.4.1"
        self.UDP_PORT = 8888
        self.NUM_TRIALS = 10
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.soc.bind(('0.0.0.0', 1444))
        self.soc.settimeout(0.001)
    
    def start_streaming(self):
        message = bytes([0, 0, 0])
        for i in range(self.NUM_TRIALS):
            try:
                self.soc.sendto(message, (self.UDP_IP, self.UDP_PORT))
                measurements = self.get_measurements()
                if measurements is None:
                    print("No data received, retrying...")
                    time.sleep(0.1)  # Pause between retries to avoid overwhelming the device
                    continue
                else:
                    print("ESP32 is alive and streaming successfully")
                    return True
            except socket.error as e:
                print(f"Error sending data: {e}")
        print("ESP32 did not respond after multiple trials.")
        return False

    def get_measurements(self):
        last_data = None
        while True:
            try:
                data, addr = self.soc.recvfrom(1024)  # Receives up to 1024 bytes
                # print(f"Received data: {data} from {addr}")
                if len(data) == 28:  # 7 floats * 4 bytes each = 28 bytes
                    last_data = data  # Save the most recent valid data packet
                else:
                    print(f"Unexpected data length: {len(data)} bytes")
                    break             
            except socket.timeout:
                break
            except socket.error as e:
                print(f"Socket error: {e}")
                return None        
        if last_data is None:
            print("No valid data received.")
            return None
        quaternion_and_gyro = struct.unpack('>7f', last_data)  # '>7f' means 7 floats in big-endian order
        print(f"Quaternion and gyro data: {quaternion_and_gyro}")        
        q = np.array(quaternion_and_gyro[0:4])  # First 4 floats are quaternion
        gyro = np.array(quaternion_and_gyro[4:7])  # Next 3 floats are gyro measurements
        return [q, gyro]

    