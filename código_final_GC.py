import math
import RPi.GPIO as GPIO # type: ignore
import time
import spidev # type: ignore
import serial # type: ignore
import folium # type: ignore
import matplotlib.pyplot as plt # type: ignore
import matplotlib.dates as mdates # type: ignore
from datetime import datetime
from selenium import webdriver # type: ignore
from selenium.webdriver.chrome.options import Options # type: ignore
from mq import * # type: ignore
import sys
from folium import plugins # type: ignore
from folium.plugins import HeatMap # type: ignore
import csv
import pandas as pd # type: ignore
from itertools import zip_longest
import urllib.request
import os
import subprocess
import shlex
import threading
import mpu6050 # type: ignore
import smbus # type: ignore
from geopy.distance import geodesic # type: ignore
from pigpio_dht import DHT11 # type: ignore

###########################################
##      Initialize Values and Sensors    ##
########################################### 

global dtn_path
global move_dir
global send_file
global file_transfer_complete
global loop_value

dtn_path = f'cp data.csv /home/pi/HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/sender/flightdata'
move_dir = f'HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/sender/'
send_file = f'./start_ltp_send_bpv6.sh'
file_transfer_complete = False
loop_value = -1

#Initialize timer between measurements while there is no gps fix
travel_time = 10

# Create an event for data acquisition completion
acquisition_complete = threading.Event()

# Create a lock for synchronizing file writes
file_lock = threading.Lock()

distance_timer = time.time()  # Timer for distance measurements

# Create empty lists for storing data
path_id = []
date = []
timestamps = []
distances1 = []
distances2 = []
distances3 = []
gps_latitudes = []
gps_longitudes = []
gps_speeds = []
temperatures = []
humidities = []
acceleration = []
gyros_pitch = []
gyros_yaw = []
temp_inside =[]
lpg_values = []
co_values = []
smoke_values = [] 
distances_kpi =[]
left_kpi = []
front_kpi = []
right_kpi = []
temp_kpi = []
temp_inside_kpi = []
humidity_kpi = []
speed_kpi = []
gyros_pitch_kpi = []
gyros_yaw_kpi =[]
lpg_kpi = []
co_kpi = []
smoke_kpi = []

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for each sensor
TRIG1 = 12
ECHO1 = 6
TRIG2 = 13
ECHO2 = 26
TRIG3 = 25
ECHO3 = 24
DHT_PIN = 5

# Set up GPIO pins as output (for trigger) or input (for echo)
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)
GPIO.setup(TRIG3, GPIO.OUT)
GPIO.setup(ECHO3, GPIO.IN)

# Initialize DHT11 sensor
DHT_SENSOR = DHT11(DHT_PIN)

# MPU-6050 I2C address, registers and their addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
TEMP_OUT_H   = 0x41

# Conversion factors
ACCEL_SCALE = 16384.0  # LSB/g
GRAVITY = 9.81         # m/s²
GYRO_SCALE = 131.0     # LSB/(°/s) for ±250°/s

# Initialize I2C (SMBus)
bus = smbus.SMBus(1)

# Write to power management register to wake up the MPU-6050
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

# Open serial port
ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)

# Enable GNSS module
ser.write(b'AT+CGNSPWR=1\r\n')
time.sleep(1)

# Wait for "OK" response from the module
response = ser.readline().decode('utf-8')
while "OK" not in response:
    response = ser.readline().decode('utf-8')

# Code execution continues only after receiving "OK" response
print("GNSS turned on successfully.")


###########################################
##    Connection / Auxiliar Functions    ##
########################################### 

def write_to_file(row_of_values, m_type, flag):
    if flag == 0:
        with open('data.csv', mode=m_type, newline='') as file:
            writer = csv.writer(file)
            writer.writerow(row_of_values)
    
    else:
        with file_lock:
            with open('data.csv', mode=m_type, newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row_of_values)

def calculate_aqi(concentration):
    breakpoints = [0, 51, 101, 151, 201, 301, 401, 501]
    index = 0
    
    while index < len(breakpoints) - 1 and concentration > breakpoints[index + 1]:
        index += 1

    c_low = breakpoints[index]
    c_high = breakpoints[index + 1]
    i_low = index * 50
    i_high = (index + 1) * 50

    aqi = ((i_high - i_low) / (c_high - c_low)) * (concentration - c_low) + i_low
    return int(aqi)

def get_classification(aqi):
    if aqi <= 50:
        return 1
    elif aqi <= 100:
        return 2
    elif aqi <= 150:
        return 3
    elif aqi <= 200:
        return 4
    
def verify_internet_connection():
    try:
        subprocess.check_output(["ping", "-c", "1", "192.168.21.63"])
        return True
    except subprocess.CalledProcessError:
        return False
        
def transfer_dtn_file():
    print(f"Defined source_root: {os.getenv('HDTN_SOURCE_ROOT')}")
    subprocess.run(dtn_path, shell=True)

    print("Sending file over DTN")
    os.chdir(move_dir)
    dispatch = subprocess.run(send_file, shell=True)
    print("DTN running...")
    if dispatch.returncode == 0:
        print("File transfer successful!")

    else:
        print(f"DTN transfer failed with return code {dispatch.returncode}")
    
    return dispatch.returncode

def read_raw_data(addr):
    # Read two bytes of data from the specified address
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    # Concatenate the two bytes
    value = (high << 8) | low
    # Convert to signed value (16-bit)
    if value > 32768:
        value = value - 65536
    return value

def get_temp_triaxial():
    temp_raw = read_raw_data(TEMP_OUT_H)
    temp_c = (temp_raw / 340.0) + 36.53
    return temp_c
        
    
###########################################
##           Sensor Functions            ##
###########################################

def measure_distance(trig_pin, echo_pin):
    # Send a 10us pulse to trigger pin to start measurement
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    # Wait for echo pin to go high and start timing
    while GPIO.input(echo_pin) == GPIO.LOW:
        start_time = time.time()

    # Wait for echo pin to go low and stop timing
    while GPIO.input(echo_pin) == GPIO.HIGH:
        end_time = time.time()

    # Calculate distance using speed of sound and time taken
    duration = end_time - start_time
    distance = duration * 34300 / 2

    return distance

#Function to get GPS information
def get_coordinates(ser):

    # Request location information
    ser.write(b'AT+CGNSINF\r\n')
    time.sleep(1)

    # Read response from module
    response = ser.read_all().decode('utf-8')

    # Split the response string into an array of values
    values = response.split(',')

    if values[3] != '' or values[4] != '':
        latitude = float(values[3])
        longitude = float(values[4])
        speed = float(values[6])
        # Print the retrieved values
        #print(f"Latitude: {latitude}")
        #print(f"Longitude: {longitude}")
        #print(f"Speed: {speed}")

        gps_latitudes.append(latitude)
        gps_longitudes.append(longitude)
        gps_speeds.append(speed)
            
    else:
        gps_latitudes.append(None)
        gps_longitudes.append(None)
        gps_speeds.append(None)

#Function to get triaxial accelerometer and gyroscope information
def calculate_triaxial_data():
    # Initial angles
    n_pitch = 0.0
    n_yaw = 0.0

    # Time of previous measurement
    previous_time = time.time()

    time.sleep(1)

    # Read current time
    present_time = time.time()
    dt = present_time - previous_time
    
    # Convert the accelerometer data to m/s²
    Ay = (read_raw_data(ACCEL_YOUT_H) / ACCEL_SCALE) * GRAVITY
    acceleration.append(abs(Ay))
    
    # Calculate pitch and yaw from gyroscope data in degrees per second
    n_pitch = (read_raw_data(GYRO_YOUT_H) / GYRO_SCALE) * dt
    n_yaw = (read_raw_data(GYRO_ZOUT_H) / GYRO_SCALE) * dt
    gyros_pitch.append(n_pitch)
    gyros_yaw.append(n_yaw)

    triax_temp = get_temp_triaxial()
    temp_inside.append(triax_temp)
    
    #print(f"Acceleration = {Ay:.2f} m/s²")
    #print(f"Pitch = {n_pitch:.2f}°, Yaw = {n_yaw:.2f}°")
    #print(f"Temperature Inside = {triax_temp:.2f}°C")
        

###########################################
##            Thread Functions           ##
###########################################

# Define a function for data acquisition and writing to a file
def data_acquisition_thread():
    loop_value = -1
    air_quality_counter = 0
    flag_start = 0

    while not os.path.exists('/home/pi/HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/sender/flightdata/data.csv'):
        loop_value += 1
        
        path_id.append(1)
        current_date = datetime.now().strftime("%Y-%m-%d")  # Get the current date
        date.append(current_date)
        current_time = datetime.now().strftime("%H:%M:%S")
        timestamps.append(current_time)
        
        # Measure distance from sensor 1
        dist1 = measure_distance(TRIG1, ECHO1)
        print("Distance from sensor 1: %.1f cm" % dist1)
        dist1 = round(dist1, 2)

        # Measure distance from sensor 2
        dist2 = measure_distance(TRIG2, ECHO2)
        print("Distance from sensor 2: %.1f cm" % dist2)
        dist2 = round(dist2, 2)

        # Measure distance from sensor 3
        dist3 = measure_distance(TRIG3, ECHO3)
        print("Distance from sensor 3: %.1f cm" % dist3)
        dist3 = round(dist3, 2)
        
        distances1.append(dist1)
        distances2.append(dist2)
        distances3.append(dist3)

        get_coordinates(ser)

        result = DHT_SENSOR.read()
        new_temp = result['temp_c']
        new_hum = result['humidity']

        temperatures.append(new_temp)
        humidities.append(new_hum)
        #print("Temperature: {:.2f}°C".format(new_temp))
        #print("Humidity: {:.2f}%".format(new_hum))

        calculate_triaxial_data()
 
        if air_quality_counter % 4 == 0:
            #Collect data from MQ-135 sensor
            mq = MQ(); # type: ignore
            perc = mq.MQPercentage()
            sys.stdout.write("\r")
            sys.stdout.write("\033[K")
            sys.stdout.write("LPG: %g ppm, CO: %g ppm, Smoke: %g ppm\n" % (perc["GAS_LPG"], perc["CO"], perc["SMOKE"]))
            sys.stdout.flush()
            time.sleep(0.1)
            lpg_values.append(perc["GAS_LPG"])
            co_values.append(perc["CO"])
            smoke_values.append(perc["SMOKE"])

            # Calculate AQI based on the concentrations
            lpg_kpi.append(get_classification(calculate_aqi(perc["GAS_LPG"])))
            co_kpi.append(get_classification(calculate_aqi(perc["CO"])))
            smoke_kpi.append(get_classification(calculate_aqi(perc["SMOKE"])))
        
        else:
            lpg_values.append(0)
            co_values.append(0)
            smoke_values.append(0)
            lpg_kpi.append(1)
            co_kpi.append(1)
            smoke_kpi.append(1)

        # Set the acquisition_complete event
        acquisition_complete.set()
        air_quality_counter += 1
        
        if flag_start != 0:    
            row_of_data = zip(path_id, date, timestamps,
                            distances1, distances2, distances3, gps_latitudes, gps_longitudes, gps_speeds,
                            temperatures, humidities, acceleration, gyros_pitch, gyros_yaw, temp_inside, lpg_values, co_values,
                            smoke_values, distances_kpi, left_kpi, front_kpi, right_kpi, temp_kpi, temp_inside_kpi,
                            humidity_kpi, speed_kpi, gyros_pitch_kpi, gyros_yaw_kpi, lpg_kpi, co_kpi, smoke_kpi)
        
            write_to_file(row_of_data, 'a', 1)
        
        flag_start = 1
        
    time.sleep(travel_time)


# Define a function for data processing and KPI calculation
def data_processing_thread():
    loop_value = -1
    flag_start = 0

    kpi_thresholds = {'distances': [50, 200, 350],
                      'speeds': [10, 15, 20],
                      'temperature_inside': [30, 20, 10],
                      'temperature_outside': [35, 25, 15],
                      'humidity': [87.5, 75, 62.5, 37.5, 25, 12.5],
                      'gyro_values': [-10, -5, -2.5, 2.5, 5, 10]}

    while not os.path.exists('/home/pi/HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/sender/flightdata/data.csv'):
        loop_value += 1

        # Wait for the acquisition_complete event
        acquisition_complete.wait()

        #################################
        #  Distances_KPIs_measurements  #
        #################################

        if distances1[loop_value] < kpi_thresholds['distances'][0] or distances2[loop_value] < kpi_thresholds['distances'][0] or distances3[loop_value] < kpi_thresholds['distances'][0]:
            distances_kpi.append(4)
        elif kpi_thresholds['distances'][0] <= distances1[loop_value] < kpi_thresholds['distances'][1] or kpi_thresholds['distances'][0] <= distances2[loop_value] < kpi_thresholds['distances'][1] or kpi_thresholds['distances'][0] <= distances3[loop_value] < kpi_thresholds['distances'][1]:
            distances_kpi.append(3)
        elif kpi_thresholds['distances'][1] <= distances1[loop_value] < kpi_thresholds['distances'][2] or kpi_thresholds['distances'][1] <= distances2[loop_value] < kpi_thresholds['distances'][2] or kpi_thresholds['distances'][1] <= distances3[loop_value] < kpi_thresholds['distances'][2]:
            distances_kpi.append(2)
        else:
            distances_kpi.append(1)

        if distances1[loop_value] < kpi_thresholds['distances'][0]:
            left_kpi.append(4)
        elif kpi_thresholds['distances'][0] <= distances1[loop_value] < kpi_thresholds['distances'][1]:
            left_kpi.append(3)
        elif kpi_thresholds['distances'][1] <= distances1[loop_value] < kpi_thresholds['distances'][2]:
            left_kpi.append(2)
        else:
            left_kpi.append(1)

        if distances2[loop_value] < kpi_thresholds['distances'][0]:
            front_kpi.append(4)
        elif kpi_thresholds['distances'][0] <= distances2[loop_value] < kpi_thresholds['distances'][1]:
            front_kpi.append(3)
        elif kpi_thresholds['distances'][1] <= distances2[loop_value] < kpi_thresholds['distances'][2]:
            front_kpi.append(2)
        else:
            front_kpi.append(1)

        if distances3[loop_value] < kpi_thresholds['distances'][0]:
            right_kpi.append(4)
        elif kpi_thresholds['distances'][0] <= distances3[loop_value] < kpi_thresholds['distances'][1]:
            right_kpi.append(3)
        elif kpi_thresholds['distances'][1] <= distances3[loop_value] < kpi_thresholds['distances'][2]:
            right_kpi.append(2)
        else:
            right_kpi.append(1)

        ####################################
        #  Temperatures_KPIs_measurements  #
        ####################################

        if temperatures[loop_value] >= kpi_thresholds['temperature_outside'][0]:
            temp_kpi.append(4)
        elif kpi_thresholds['temperature_outside'][1] <= temperatures[loop_value] < kpi_thresholds['temperature_outside'][0]:
            temp_kpi.append(3)
        elif kpi_thresholds['temperature_outside'][2] <= temperatures[loop_value] < kpi_thresholds['temperature_outside'][1]:
            temp_kpi.append(2)
        else:
            temp_kpi.append(1)

        if temp_inside[loop_value] >= kpi_thresholds['temperature_inside'][0]:
            temp_inside_kpi.append(4)
        elif kpi_thresholds['temperature_inside'][1] <= temp_inside[loop_value] < kpi_thresholds['temperature_inside'][0]:
            temp_inside_kpi.append(3)
        elif kpi_thresholds['temperature_inside'][2] <= temp_inside[loop_value] < kpi_thresholds['temperature_inside'][1]:
            temp_inside_kpi.append(2)
        else:
            temp_inside_kpi.append(1)

        ####################################
        #     Humidity_KPI_measurements    #
        ####################################

        if humidities[loop_value] >= kpi_thresholds['humidity'][0] or humidities[loop_value] <= kpi_thresholds['humidity'][5]:
            humidity_kpi.append(4)
        elif (kpi_thresholds['humidity'][1] <= humidities[loop_value] < kpi_thresholds['humidity'][0]) or (kpi_thresholds['humidity'][4] <= humidities[loop_value] < kpi_thresholds['humidity'][5]):
            humidity_kpi.append(3)
        elif (kpi_thresholds['humidity'][2] <= humidities[loop_value] < kpi_thresholds['humidity'][1]) or (kpi_thresholds['humidity'][3] <= humidities[loop_value] < kpi_thresholds['humidity'][4]):
            humidity_kpi.append(2)
        else:
            humidity_kpi.append(1)

        ####################################
        #      Speed_KPI_measurements      #
        ####################################

        if gps_speeds[loop_value] >= kpi_thresholds['speeds'][2]:
            speed_kpi.append(4)
        elif kpi_thresholds['speeds'][1] <= gps_speeds[loop_value] < kpi_thresholds['speeds'][2]:
            speed_kpi.append(3)
        elif kpi_thresholds['speeds'][0] <= gps_speeds[loop_value] < kpi_thresholds['speeds'][1]:
            speed_kpi.append(2)
        else:
            speed_kpi.append(1)

        ####################################
        #    Gyroscope_KPI_measurements    #
        ####################################

        if gyros_pitch[loop_value] <= kpi_thresholds['gyro_values'][0] or gyros_pitch[loop_value] >= kpi_thresholds['gyro_values'][5]:
            gyros_pitch_kpi.append(4)
        elif (kpi_thresholds['gyro_values'][0] < gyros_pitch[loop_value] <= kpi_thresholds['gyro_values'][1]) or (kpi_thresholds['gyro_values'][4] <= gyros_pitch[loop_value] < kpi_thresholds['gyro_values'][5]):
            gyros_pitch_kpi.append(3)
        elif (kpi_thresholds['gyro_values'][1] < gyros_pitch[loop_value] <= kpi_thresholds['gyro_values'][2]) or (kpi_thresholds['gyro_values'][3] <= gyros_pitch[loop_value] < kpi_thresholds['gyro_values'][4]):
            gyros_pitch_kpi.append(2)
        else:
            gyros_pitch_kpi.append(1)

        if gyros_yaw[loop_value] <= kpi_thresholds['gyro_values'][0] or gyros_yaw[loop_value] >= kpi_thresholds['gyro_values'][5]:
            gyros_yaw_kpi.append(4)
        elif (kpi_thresholds['gyro_values'][0] < gyros_yaw[loop_value] <= kpi_thresholds['gyro_values'][1]) or (kpi_thresholds['gyro_values'][4] <= gyros_yaw[loop_value] < kpi_thresholds['gyro_values'][5]):
            gyros_yaw_kpi.append(3)
        elif (kpi_thresholds['gyro_values'][1] < gyros_yaw[loop_value] <= kpi_thresholds['gyro_values'][2]) or (kpi_thresholds['gyro_values'][3] <= gyros_yaw[loop_value] < kpi_thresholds['gyro_values'][4]):
            gyros_yaw_kpi.append(2)
        else:
            gyros_yaw_kpi.append(1)

        # Clear the acquisition_complete event
        acquisition_complete.clear()

        if flag_start != 0:    
            row_of_data = zip(path_id, date, timestamps,
                            distances1, distances2, distances3, gps_latitudes, gps_longitudes, gps_speeds,
                            temperatures, humidities, acceleration, gyros_pitch, gyros_yaw, temp_inside, lpg_values, co_values,
                            smoke_values, distances_kpi, left_kpi, front_kpi, right_kpi, temp_kpi, temp_inside_kpi,
                            humidity_kpi, speed_kpi, gyros_pitch_kpi, gyros_yaw_kpi, lpg_kpi, co_kpi, smoke_kpi)
        
            write_to_file(row_of_data, 'a', 1)
        
        flag_start = 1
    
    time.sleep(travel_time)


def dtn_send_thread():

    os.environ['HDTN_SOURCE_ROOT'] = '/home/pi/HDTN'

    while True:
        acquisition_complete.wait()

        # Check internet connection with receiving node
        if verify_internet_connection():
            file_transfer_complete = True
            dtn_transfer_start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"Internet connection available. Initiating DTN transfer at {dtn_transfer_start_time}.")
            transfer_success = transfer_dtn_file()

            if transfer_success == 0:
                print("Transfer completed!")
                acquisition_complete.clear()
                return
            else:
                print(f"DTN transfer failed with return code {transfer_success}")

        else:
            print("No internet connection available.")



###########################################
##             Main Function             ##
###########################################

if __name__ == "__main__":

    print("Starting...")

    # Create a CSV file and write the header
    header_row = ["Ride", "Date", "Timestamp",
                  "Distance1", "Distance2", "Distance3", "Latitude", "Longitude",  "Speed",
                  "Temperature", "Humidity", "Acceleration", "Pitch", "Yaw", "Inside Temperature", "LPG Concentration", "CO Concentration",
                  "Smoke Concentration", "ObstaclesKPI", "LeftKPI", "FrontKPI", "RightKPI", "TemperatureKPI", "InsideTemperatureKPI",
                  "HumidityKPI", "SpeedKPI", "PitchKPI", "YawKPI", "LPGKPI", "COKPI", "SmokeKPI"]

    write_to_file(header_row, 'w', 0)

    # Create the aquisition and processing threads
    acquisition_thread = threading.Thread(target=data_acquisition_thread)
    processing_thread = threading.Thread(target=data_processing_thread)

    # Start the acquisition thread first
    acquisition_thread.start()
    processing_thread.start()

    dtn_send_thread()
    
    acquisition_thread.join()  # Wait for the acquisition thread to complete
    processing_thread.join()  # Wait for the processing thread to complete

    print("Program Finished.")

    #GPIO cleanup
    GPIO.cleanup()

    # Disable GNSS module
    ser.write(b'AT+CGNSPWR=0\r\n')
    time.sleep(1)

    # Close serial port
    ser.close()