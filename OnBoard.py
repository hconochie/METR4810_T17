#Default imports
import time
import random

# Radio imports
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import spidev

#Vision imports
from imutils.video.pivideostream import PiVideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import imutils
import cv2
import sys
import numpy as np

## Sensor import
import smbus
from time import sleep


GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]


########## Sensor Init ###############
# Get I2C bus
bus = smbus.SMBus(1)	

# I2C address of the device
MPL3115A2_DEFAULT_ADDRESS			= 0x60	#MPL3115A2 device address
Device_Address = 0x68   # MPU6050 device address
 
# MPL3115A2 Regster Map
MPL3115A2_REG_STATUS				= 0x00 # Sensor status Register
MPL3115A2_REG_PRESSURE_MSB			= 0x01 # Pressure data out MSB
MPL3115A2_REG_PRESSURE_CSB			= 0x02 # Pressure data out CSB
MPL3115A2_REG_PRESSURE_LSB			= 0x03 # Pressure data out LSB
MPL3115A2_REG_TEMP_MSB				= 0x04 # Temperature data out MSB
MPL3115A2_REG_TEMP_LSB				= 0x05 # Temperature data out LSB
MPL3115A2_REG_DR_STATUS				= 0x06 # Data Ready status registe
MPL3115A2_OUT_P_DELTA_MSB			= 0x07 # Pressure data out delta MSB
MPL3115A2_OUT_P_DELTA_CSB			= 0x08 # Pressure data out delta CSB
MPL3115A2_OUT_P_DELTA_LSB			= 0x09 # Pressure data out delta LSB
MPL3115A2_OUT_T_DELTA_MSB			= 0x0A # Temperature data out delta MSB
MPL3115A2_OUT_T_DELTA_LSB			= 0x0B # Temperature data out delta LSB
MPL3115A2_REG_WHO_AM_I				= 0x0C # Device Identification Register
MPL3115A2_PT_DATA_CFG				= 0x13 # PT Data Configuration Register
MPL3115A2_CTRL_REG1					= 0x26 # Control Register-1
MPL3115A2_CTRL_REG2					= 0x27 # Control Register-2
MPL3115A2_CTRL_REG3					= 0x28 # Control Register-3
MPL3115A2_CTRL_REG4					= 0x29 # Control Register-4
MPL3115A2_CTRL_REG5					= 0x2A # Control Register-5
 
# MPL3115A2 PT Data Configuration Register
MPL3115A2_PT_DATA_CFG_TDEFE			= 0x01 # Raise event flag on new temperature data
MPL3115A2_PT_DATA_CFG_PDEFE			= 0x02 # Raise event flag on new pressure/altitude data
MPL3115A2_PT_DATA_CFG_DREM			= 0x04 # Generate data ready event flag on new pressure/altitude or temperature data
 
# MPL3115A2 Control Register-1 Configuration
MPL3115A2_CTRL_REG1_SBYB			= 0x01 # Part is ACTIVE
MPL3115A2_CTRL_REG1_OST				= 0x02 # OST Bit ACTIVE
MPL3115A2_CTRL_REG1_RST				= 0x04 # Device reset enabled
MPL3115A2_CTRL_REG1_OS1				= 0x00 # Oversample ratio = 1
MPL3115A2_CTRL_REG1_OS2				= 0x08 # Oversample ratio = 2
MPL3115A2_CTRL_REG1_OS4				= 0x10 # Oversample ratio = 4
MPL3115A2_CTRL_REG1_OS8				= 0x18 # Oversample ratio = 8
MPL3115A2_CTRL_REG1_OS16			= 0x20 # Oversample ratio = 16
MPL3115A2_CTRL_REG1_OS32			= 0x28 # Oversample ratio = 32
MPL3115A2_CTRL_REG1_OS64			= 0x30 # Oversample ratio = 64
MPL3115A2_CTRL_REG1_OS128			= 0x38 # Oversample ratio = 128
MPL3115A2_CTRL_REG1_RAW				= 0x40 # RAW output mode
MPL3115A2_CTRL_REG1_ALT				= 0x80 # Part is in altimeter mod
MPL3115A2_CTRL_REG1_BAR				= 0x00 # Part is in barometer mode
 
# ###Sensor functions###
def control_alt_config():
	"""Select the Control Register-1 Configuration from the given provided value"""
	CONTROL_CONFIG = (MPL3115A2_CTRL_REG1_SBYB | MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT)
	bus.write_byte_data(MPL3115A2_DEFAULT_ADDRESS, MPL3115A2_CTRL_REG1, CONTROL_CONFIG)

def data_config():
	"""Select the PT Data Configuration Register from the given provided value"""
	DATA_CONFIG = (MPL3115A2_PT_DATA_CFG_TDEFE | MPL3115A2_PT_DATA_CFG_PDEFE | MPL3115A2_PT_DATA_CFG_DREM)
	bus.write_byte_data(MPL3115A2_DEFAULT_ADDRESS, MPL3115A2_PT_DATA_CFG, DATA_CONFIG)

def read_alt_temp():
	"""Read data back from MPL3115A2_REG_STATUS(0x00), 6 bytes
	status, tHeight MSB, tHeight CSB, tHeight LSB, temp MSB, temp LSB"""
	data = bus.read_i2c_block_data(MPL3115A2_DEFAULT_ADDRESS, MPL3115A2_REG_STATUS, 6)

	# Convert the data to 20-bits
	tHeight = ((data[1] * 65536) + (data[2] * 256) + (data[3] & 0xF0)) / 16
	temp = ((data[4] * 256) + (data[5] & 0xF0)) / 16

	altitude = tHeight / 16.0
	cTemp = temp / 16.0
	fTemp = cTemp * 1.8 + 32

	return {'a' : altitude, 'c' : cTemp, 'f' : fTemp}

def control_pres_config():
	"""Select the Control Register-1 Configuration from the given provided value"""
	CONTROL_CONFIG = (MPL3115A2_CTRL_REG1_SBYB | MPL3115A2_CTRL_REG1_OS128)
	bus.write_byte_data(MPL3115A2_DEFAULT_ADDRESS, MPL3115A2_CTRL_REG1, CONTROL_CONFIG)

def read_pres():
	"""Read data back from MPL3115A2_REG_STATUS(0x00), 4 bytes
	status, pres MSB, pres CSB, pres LSB"""
	data = bus.read_i2c_block_data(MPL3115A2_DEFAULT_ADDRESS, MPL3115A2_REG_STATUS, 4)

	# Convert the data to 20-bits
	pres = ((data[1] * 65536) + (data[2] * 256) + (data[3] & 0xF0)) / 16
	pressure = (pres / 4.0) / 1000.0

	return {'p' : pressure}
 

counter = 1
 



#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


MPU_Init()
#### End sensor Init ####


# ###Radio Init###
radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)
radio.setPayloadSize(32)
radio.setChannel(0x60)

radio.setDataRate(NRF24.BR_2MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1, pipes[1])
radio.printDetails()

radio.startListening()
# ###END Radio Init###


#### Vision Functions ###
class PiVideoStream:
	def __init__(self, resolution = (1216,1200), framerate = 30):
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.framerate = framerate
		self.rawCapture = PiRGBArray(self.camera, size=resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture,
			format = "bgr", use_video_port = False)
			
		self.frame = None
		self.stopped = False
		
	def start(self):
		Thread(target=self.update, args=()).start()
		return self
		
	def update(self):
		#Keep looping until the thread has stopped
		for f in self.stream:
			self.frame = f.array
			self.rawCapture.truncate(0)
			
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.camera.close()
				return
				
	def read(self):
		return self.frame
		
	def stop(self):
		self.stopped = True


def initialize_camera():
	#Load the aruco marker dictionary
	tic = time.perf_counter()
	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
	arucoParams = cv2.aruco.DetectorParameters_create()
	
	arucoParams.minMarkerPerimeterRate = 0.08
	arucoParams.polygonalApproxAccuracyRate = 0.1 	
	arucoParams.minDistanceToBorder = 2 
	arucoParams.perspectiveRemovePixelPerCell = 4 
	arucoParams.perspectiveRemoveIgnoredMarginPerCell = 0.2
	arucoParams.errorCorrectionRate = 1.2
	arucoParams.cornerRefinementMaxIterations = 30 
	
	if print_to_terminal:
		print("[1] Loaded dictionaries")
		if print_time_of_function:
			toc = time.perf_counter()
			print(f"Time to load dictionary = {toc - tic:0.4f} seconds")

			
	#Load the camera
	tic = time.perf_counter()
	vid_stream = PiVideoStream().start()
	time.sleep(2)
	
	if print_to_terminal:
		print("[2] Set camera")
		if print_time_of_function:
			toc = time.perf_counter()
			print(f"Time to load camera = {toc - tic:0.4f} seconds")
			
	return arucoDict,arucoParams,vid_stream

def capture_image(arucoDict,arucoParams, vid_stream):
	#Capture an image
	tic = time.perf_counter()
	
	start_x = vid_stream.camera.resolution[0]/2
	start_y = vid_stream.camera.resolution[1]/2
	
	if print_to_terminal:
		print("[3] Take a photo")
		if print_time_of_function:
			toc = time.perf_counter()
			print("Time to take a photo = {toc - tic:0.4f} seconds")
				
	#Find aruco Markers
	tic = time.perf_counter()
	#run for a max number of frames or until marker is found
	max_frames = 1
	desired_markers = [72]
	
	frame_num = 1
	no_marker = True
	x_out = 0.5
	y_out = 0.5
	
	while (frame_num <= max_frames) and (no_marker):
		frame = vid_stream.read()
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
		#detect ArUco markers in the input frame
		(corners, ids, rejected) = cv2.aruco.detectMarkers(frame,arucoDict, parameters=arucoParams)
		# verify *at least* one of the 2 desired ArUco marker were detected
		if len(corners) > 0 and desired_markers in ids:
			# flatten the ArUco IDs list
			ids = ids.flatten()
			# loop over the detected ArUCo corners
			for (markerCorner, markerID) in zip(corners, ids):
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners
				# convert each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))

				#ArUco marker center position
				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			

				start = (start_x, start_y)
				end = (cX, cY)

				x_out = (cX)/(2*start_x)
				y_out = (cY)/(2*start_y)
			
				#-1 to 1 output
				#x_out = (cX - start_x)/start_x
				#y_out = (cY - start_y)/start_y
				
				if print_to_terminal:
					print("[4] Detected Marker")
				no_marker = False
		frame_num += 1
	
	if print_time_of_function:
		toc = time.perf_counter()
		print(f"Time to Detect markers = {toc - tic:0.4f} seconds")
	return x_out, y_out
	
def show_image(direction, vid_stream):
	w = 480
	h = 360
	frame = vid_stream.read()
	frame = cv2.resize(frame, (w,h))

	(x,y) = direction
	start = (int(w/2), int(h/2))
	end = (int(w*x),int(h*y))
	cv2.line(frame, start, end, (255,255,255), 2)
	
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

#Print data for controlling timing
print_to_terminal = False
#requires print_to_terminal
print_time_of_function = False

arucoDict, arucoParams, vid_stream = initialize_camera()


# ###Get sensor data###
def getTemp():
    control_alt_config()
    data_config()
    alt = read_alt_temp()
    return str(round(np.float16(alt['c']),2)) ## returns degrees celcius

def getPressure():
    control_pres_config()
    pressure = (read_pres())
    return str(round(np.float16(pressure['p']),2))

def getAccel():
    #Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    #Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = round(np.float16(acc_x/16384.0),2)
    Ay = round(np.float16(acc_y/16384.0),2)
    Az = round(np.float16(acc_z/16384.0),2)

    return str(Ax), str(Ay), str(Az)

def getOrien():
    #Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0

    orientation = [round(Gx,2), round(Gy, 2), round(Gz, 2)]
    return orientation

# ###Sending Data###
def sendData(tempID, temp, pressureID, pressure):
    radio.stopListening()
    time.sleep(0.25)
    message = list(tempID) + list(temp)+ list(pressureID) + list(pressure)
    print("About to send message")
    print("message: ", message)
    radio.write(message)
    print("send data to master")
    radio.startListening()

def sendAccel(axID, ax, ayID, ay, azID, az):
    radio.stopListening()
    time.sleep(0.25)
    message = list(axID) + list(ax) + list(ayID) + list(ay) + list(azID) + list(az)
    print("About to send message")
    radio.write(message)
    print("send data to master")
    radio.startListening()

def sendOdom(rollID, roll, pitchID, pitch, yawID, yaw):
    radio.stopListening()
    time.sleep(0.25)
    message =list(rollID) + list(roll) + list(pitchID) + list(pitch) + list(yawID) + list(yaw)
    print("About to send message")
    radio.write(message)
    print("send data to master")
    radio.startListening()

def sendLook(xPosID, xPos, yPosID, ypos):
    radio.stopListening()
    time.sleep(0.25)
    message = list(xPosID) + list(xPos) + list(yPosID) + list(yPos)
    print("About to send message")
    radio.write(message)
    print("send data to master")
    radio.startListening()

####### MAIN LOOP #######
while True:
    ackPL = [1]
    radio.writeAckPayload(1, ackPL, len(ackPL))
    # print("radio.available: ",radio.available(0))
    while not radio.available(0):
        time.sleep(1/100)
       # print("sleeping...")

    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())
    print("Received: {}".format(receivedMessage))

    print("Translating the receivedMessage into unicode charaters...")
    string = ""
    for n in receivedMessage:
        # decode into standard unicode set
        if (n>=32 and n <=126):
            string += chr(n)
    print(string)
    
    command = string
    if command == "GET_DATA":
        print("We should get the DATA!")
        k = random.randint(0, 6)
        tempID = "t_"
        temp = getTemp()
        pressureID = "_p_"
        pressure = getPressure()
        sendData(tempID, temp, pressureID, pressure)

    if command == "GET_ACC":
        xacc, yacc, zacc = getAccel()
        axID = "ax_"
        ayID = "_ay_"
        azID = "_az_"
        sendAccel(axID, xacc, ayID, yacc, azID,  zacc)
    
    if command == "GET_POS":
        print("We should get the POSITION!")
        k = random.randint(0, 6)
        delta_xID = "deltax_"
        delta_yID = "_deltay_"
        delta_zID = "_deltaz_"
        delta = getDelta(k)
        delta_x = str(delta[0])
        delta_y = str(delta[1])
        delta_z = str(delta[2])
        sendPos(delta_xID, delta_x, delta_yID, delta_y, delta_zID, delta_z)
    
    if command == "GET_ODOM":
        print("We should get the Odometry!")
        k = random.randint(0, 6)
        rollID = "roll_"
        pitchID = "_pitch_"
        yawID = "_yaw_"
        orien = getOrien()
        roll = str(orien[0])
        pitch = str(orien[1])
        yaw = str(orien[2])
        sendOdom(rollID, roll, pitchID, pitch, yawID, yaw)

    if command == "POWER_CYCLE":
        print("My battery is low and its getting dark")
        GPIO.output(15, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(15, GPIO.LOW)

    if command == "LAND":
        print("Now attempting landing...")
        GPIO.output(18, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(18, GPIO.LOW)

    if command == "LOOK":
        x, y  = capture_image(arucoDict,arucoParams, vid_stream)
        xPos = str(round(x,3))
        yPos = str(round(y,3))
        #show_image(direction, vid_stream)
        xPosID = "xPos_"
        yPosID = "_yPos_"
        sendLook(xPosID, xPos, yPosID, yPos)

    command = ""

    radio.writeAckPayload(1, ackPL, len(ackPL))
    print("Loaded payload reply of {}".format(ackPL))
    
 
