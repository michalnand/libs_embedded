import time
import json
import numpy
import matplotlib.pyplot as plt
from JsonSerialPortParser import *



serial_port_name = "/dev/tty.usbmodem111303"
serial_port      = JsonSerialPortParser(serial_port_name)


distance_raw        = []
distance_filtered   = []

while True:
    serial_port.process()
    if serial_port.udpated():
        json_data = serial_port.get()
        print("new data ", len(distance_raw))
        print(json_data)


        distance_raw.append(float(json_data["distance_r"]))
        distance_filtered.append(float(json_data["distance_f"]))

        if len(distance_raw) > 200:
            break
    else: 
        time.sleep(0.1)




plt.plot(distance_raw, label="raw data", color="blue")
plt.plot(distance_filtered, label="filtered data", color="purple")
plt.xlabel("sample [n]")
plt.ylabel("distance [mm]")
plt.legend()
plt.show()