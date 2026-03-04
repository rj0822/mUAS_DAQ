# Data Acquisition Program for Multi-Sensor Multi-Hazard Monitoring Unmanned Aircraft System (mUAS)

# Author: Ruipu Ji (r5ji@ucsd.edu)
# Affiliation: Department of Structural Engineering, University of California San Diego

# Python version: 3.10.11
# Required packages: numpy, opencv, pyserial, depthai

# Test platform:
# (1) Ultrasonic anemometer x 2: LI-COR LI550-P
# (2) Particulate matter sensor x 1: Honeywell HPMA115C0-004
# (3) RGB camera x 1: Luxonis OAK-1W IMX378
# (4) Onboard computer: Intel NUC7i5BNH
# (5) UAV platform: Freefly Alta-X (with Tattu 12S 16Ah battery set)

import depthai as dai
import datetime
import subprocess
import cv2
import numpy as np
import serial
import threading
import time
from datetime import datetime

# ---------- FUNCTION: SERIAL DATA COLLECTION. ----------
def ReadSerialPort(PortName, Config, StopEvent):
    # Open the serial port given the port name and baudrate.
    SerialObject = serial.Serial(PortName, Config['baudrate'])

    # Check if the extension of the output file is '.txt'.
    # .txt file: Serial data in ASCII format. Data are written as strings to the file.
    # .bin file: Serial data in binary format. Data are written as bytes to the file.
    # Output mode: 'a' = append; 'ab' = append with binary format.
    is_text = Config['filename'].endswith('.txt') 
    Mode = 'a' if is_text else 'ab'

    # Create a byte array in the buffer to temporarily store the binary data.
    buffer = bytearray()

    try:
        with open(Config['filename'], Mode) as File:
            while not StopEvent.is_set():
                # Read all the available data in the serial buffer.
                # If no data available, then read 1 byte instead. This could avoid idle loops and reduce CPU usage.
                Data = SerialObject.read(SerialObject.in_waiting or 1)

                # If 'Data' is emtpy, continue the loop without writing anything to the file.
                # In practice this should not happen and this line is added just to prevent program crushing.
                if not Data:
                    continue

                # Serial data in ASCII format will be written as strings to the output .txt file.
                if is_text:
                    # If 'Data' is empty (actually should not happen, just to prevent program crushing) or only one byte is read, continue the loop without writing anything to the file.
                    if len(Data)<=1:
                        continue
                    else:
                        Timestamp = time.time()  # Read the timestamp from current computer timestamp. The computer timestamp is synchronized with the UTC time based on GNSS.
                        TimestampString = datetime.fromtimestamp(Timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] # Create a timestamp string with millisecond accuracy.
                        DecodedData = Data.decode(errors='replace') # Convert any non-string characters into string format (prevent program crushing).
                        DecodedData = DecodedData.rstrip('\r\n') # Keep only the string part in the decoded data (for output format).
                        DataLine = f'{TimestampString} {DecodedData}\n' # Create a the final output data string combining the timestamp string with the decoded data.
                        File.write(DataLine) # Write the output string to the output .txt file.
                        File.flush() # Make sure the data in the buffer is written to the file.
                        print(f'{PortName} → {DataLine}', end='') # Display the data in the terminal.           
                
                # Binary data will be written to the output .bin file with the timestamp.
                else:
                    # Extract the binary data for every 32 bytes.
                    buffer.extend(Data)  
                    while len(buffer) >= 32:
                        chunk = buffer[:32]
                        del buffer[:32]

                        Timestamp = time.time()  # Read the timestamp from current computer timestamp. The computer timestamp is synchronized with the UTC time based on GNSS.
                        TimestampString = datetime.fromtimestamp(Timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] # Create a timestamp string with millisecond accuracy.

                        # Decode the binary data into numbers.
                        if chunk[0] == 0x42 and chunk[1] == 0x4D:
                            FrameLength = (chunk[2] << 8) | chunk[3]
                            if FrameLength == 28:
                                pm1_0 = (chunk[4] << 8) | chunk[5]
                                pm2_5 = (chunk[6] << 8) | chunk[7]
                                pm4_0 = (chunk[8] << 8) | chunk[9]
                                pm10_0  = (chunk[10] << 8) | chunk[11]
                                DataLine = f'{TimestampString} {pm1_0} {pm2_5} {pm4_0} {pm10_0}' # Create a the final output data string combining the timestamp string with the decoded data.
                                print(f'{PortName} → {DataLine}') # Display the data in the terminal. 
                            else:
                                print(f'{PortName} → {TimestampString} INVALID FRAME LENGTH!')
                        else:
                            print('INVALID FRAME HEAD!')

                        # Write the output data to .bin file.
                        File.write(TimestampString.encode('utf-8') + b' ' + chunk + b'\n') # Write the final output (timestamp + data + '\n') in binary format in the output .bin file.
                        File.write(DataLine.encode('utf-8') + b'\n') # Write the final output in ASCII string format in the output .bin file.
                        File.flush() # Make sure the data in the buffer is written to the file.
                       
    finally:
        print('SERIAL CLOSED')
        SerialObject.close()

# ---------- FUNCTION: CONVERT RAW .H265 FILE INTO .MP4 FILE FOR RGB CAMERA. ----------     
def VideoFormatConverter(InputFileName, OutputFileName, fps):
    cmd = ['ffmpeg', '-y', '-framerate', str(fps), '-i', InputFileName, '-c', 'copy', OutputFileName]
    subprocess.run(cmd, check=True)

# ---------- FUNCTION: RGB CAMERA RECORDING (LUXONIS OAK 1-W). ----------
def ReadRGBCamera(TestStartTimeString, StopEvent):
    # Camera initialization.
    Pipeline = dai.Pipeline()
    Camera = Pipeline.create(dai.node.ColorCamera)
    Camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    Camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
    Camera.setVideoSize(3840, 2160)
    Camera.setPreviewSize(480, 270)
    Camera.setFps(30)
    Camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    Camera.setInterleaved(False)

    VideoEncoder = Pipeline.create(dai.node.VideoEncoder)
    VideoEncoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
    Camera.video.link(VideoEncoder.input)

    OutputStream = Pipeline.create(dai.node.XLinkOut)
    OutputStream.setStreamName('video')
    VideoEncoder.bitstream.link(OutputStream.input)

    PreviewStream = Pipeline.create(dai.node.XLinkOut)
    PreviewStream.setStreamName('preview')
    Camera.preview.link(PreviewStream.input)

    # Camera recording and live stream control.
    with dai.Device(Pipeline) as device:
        OutputQueue = device.getOutputQueue(name='video', maxSize=30, blocking=True)
        PreviewQueue = device.getOutputQueue(name='preview', maxSize=5, blocking=False)

        # Define the output file names for the RGB camera.
        FileName_Timestamp = '3_Camera_RGB/Timestamp/Timestamp_' + TestStartTimeString + '.txt'
        FileName_H265 = '3_Camera_RGB/H265/Video_' + TestStartTimeString + '.H265'
        FileName_MP4 = '3_Camera_RGB/MP4/Video_' + TestStartTimeString + '.MP4'

        with open(FileName_H265, 'wb') as Output_H265, open(FileName_Timestamp, 'a') as Output_Timestamp:
            # Initialize the total number of frames to 0.
            NumOfFrames = 0

            # Raw output collection and live preview.
            try:
                while not StopEvent.is_set():
                    # Create a timestamp for the current frame with millisecond accuracy.
                    Timestamp = time.time() 
                    TimestampString = datetime.fromtimestamp(Timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

                    # Collect the raw output from queue and write to the output .H265 file.
                    OutputPacket = OutputQueue.get()
                    Output_H265.write(OutputPacket.getData())
                    # Output_H265.flush()

                    # Write the timestamp into the output .txt file.
                    Output_Timestamp.write(f'{TimestampString}\n')
                    # Output_Timestamp.flush()

                    # Update the total number of frames.
                    NumOfFrames += 1

                    # Set up the live preview.
                    if NumOfFrames % 30 == 0:
                        Frame = PreviewQueue.get().getCvFrame()
                        cv2.imshow('LIVE PREVIEW', Frame)
                        if cv2.waitKey(1) == ord('q'):
                            StopEvent.set()
                            break
            
            # Close the live output window when collection is completed.
            finally:
                cv2.destroyAllWindows()

        print('RGB CAMERA → RAW .H265 OUTPUT COLLECTION COMPLETED.')
        
        # Convert the raw .H265 file into .MP4 file.
        # print('RGB CAMERA → CONVERSION TO .MP4 FILE IN PROGRESS.')
        # VideoFormatConverter(FileName_H265, FileName_MP4, 30)
        # print('RGB CAMERA → .MP4 VIDEO SAVED: ', FileName_MP4)

# ---------- MAIN PROGRAM: CONTROL FOR ALL SENSORS DATA COLLECTION. ----------
# Obtain the current computer timestamp and create a string for time information in the output file names.
# Timestamp format in the output file names: 20250716-103015 --> 2025-07-16-10:30:15.
TestStartTime = time.time()
TestStartTimeString = datetime.fromtimestamp(TestStartTime).strftime('%Y%m%d-%H%M%S%f')[:-6]

# Define the serial port objects and output file names.
SerialPorts = {
    'COM1': {'baudrate': 230400, 'filename': '1_SerialData/Serial_' + TestStartTimeString + '_COM1_LI550P-1.txt'},
    'COM2': {'baudrate': 230400, 'filename': '1_SerialData/Serial_' + TestStartTimeString + '_COM2_LI550P-2.txt'},
    'COM3': {'baudrate': 9600,   'filename': '1_SerialData/Serial_' + TestStartTimeString + '_COM3_HPMA115C0-004.bin'}
}

# Initialization of a thread variable to store each thread for sensor data collection.
Threads = []

# Initialization of a thread event to terminate the data collection.
StopEvent = threading.Event()

# Serial data collection threads setup (LI-550Px2 & HPMA115C0-004).
for PortName, Config in SerialPorts.items():
    ThreadSerial = threading.Thread(target=ReadSerialPort, args=(PortName, Config, StopEvent))
    ThreadSerial.daemon = True
    ThreadSerial.start()
    Threads.append(ThreadSerial)

# RGB camera data collection thread setup (Luxonis OAK 1-W).
ThreadRGB = threading.Thread(target=ReadRGBCamera, args=(TestStartTimeString, StopEvent))
ThreadRGB.daemon = True
ThreadRGB.start()
Threads.append(ThreadRGB)

# Termination of data collection: Press CTRL+C (KeyboardInterrupt).
try:
    # Detect the KeyboardInterrupt every 1 second.
    while not StopEvent.is_set():
        time.sleep(1)
except KeyboardInterrupt:
    print("DATA COLLECTION STOPPED.")
    StopEvent.set()

# Wait until each thread is finished.
for t in Threads:
    t.join()

print("DATA COLLECTION COMPLETED. ALL DATA SAVED TO DISK.")




