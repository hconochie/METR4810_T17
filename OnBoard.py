#Default imports
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev
import random

#Hunter H imports
from imutils.video.pivideostream import PiVideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import imutils
import cv2
import sys

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]

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


#### Start of Hunter H. Vision stuff
class PiVideoStream:
	def __init__(self, resolution = (1920,1080), framerate = 30):
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.framerate = framerate
		self.rawCapture = PiRGBArray(self.camera, size=resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture,
			format = "bgr", use_video_port = True)
			
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
	
	if print_to_terminal:
		print("[1] Loaded dictionaries")
		if print_time_of_function:
			toc = time.perf_counter()
			print(f"Time to load dictionary = {toc - tic:0.4f} seconds")

			
	#Load the camera
	tic = time.perf_counter()
	vid_stream = PiVideoStream().start()
	
	if print_to_terminal:
		print("[2] Set camera")
		if print_time_of_function:
			toc = time.perf_counter()
			print(f"Time to load camera = {toc - tic:0.4f} seconds")
			
	return arucoDict,arucoParams,vid_stream


### Vision
def capture_image(arucoDict,arucoParams, vid_stream):
	#Capture an image
	tic = time.perf_counter()
	frame = vid_stream.read()
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	start_x = vid_stream.camera.resolution[0]/2
	start_y = vid_stream.camera.resolution[1]/2
	
	if print_to_terminal:
		print("[3] Take a photo")
		if print_time_of_function:
			toc = time.perf_counter()
			print(f"Time to take a photo = {toc - tic:0.4f} seconds")
			
			
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
	return (x_out, y_out)
	
	
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


def getTemp(k):
    temp = [10, 15, 18, 25, 12, 6, 30]
    return str(temp[k])

def getPressure(k):
    pressure = [1, 2, 1.5, 1, 1, 2, 1.5]
    return str(pressure[k])

def getAccel(k):
    accel = [0, 0.1, 0.3, 0.4, 0.1, 0.2, 0.5]
    return str(accel[k])

def getDelta(k):
    delta = [[1  ,  2,  1],
             [0.1,  1,  1],
             [0  ,  0,  0],
             [1  ,  1,0.3],
             [0.2,0.1,0.2],
             [0  ,0.9,  2],
             [1  ,  2,0.4]]
    return delta[k]

def getOrien(k):
    orientation = [[100,120,100],
                   [105, 90, 15],
                   [ 10, 90, 20],
                   [270, 80, 20],
                   [200,175, 60],
                   [ 30, 30, 35],
                   [102, 30, 25]]
    return orientation[k]

def sendData(tempID, temp, pressureID, pressure, accelID, accel):
    radio.stopListening()
    time.sleep(0.25)
    message = list(tempID) + list(temp)+ list(pressureID) + list(pressure) + list(accelID) + list(accel)
    print("About to send message")
    radio.write(message)
    print("send data to master")
    radio.startListening()

def sendPos(delta_xID, delta_x, delta_yID, delta_y, delta_zID, delta_z):
    radio.stopListening()
    time.sleep(0.25)
    message = list(delta_xID) + list(delta_x) + list(delta_yID) + list(delta_y) + list(delta_zID) + list(delta_z)
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
        tempID = "temp_"
        temp = getTemp(k)
        pressureID = "_pressure_"
        pressure = getPressure(k)
        accelID = "_accel_"
        accel = getAccel(k)
        sendData(tempID, temp, pressureID, pressure, accelID, accel)
    
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
        orien = getOrien(k)
        roll = str(orien[0])
        pitch = str(orien[1])
        yaw = str(orien[2])
        sendOdom(rollID, roll, pitchID, pitch, yawID, yaw)

    if command == "ABORT":
        print("ABORT THE MISSION ARRGHHH")
        GPIO.output(23, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(23, GPIO.LOW)

    if command == "LAND":
        print("Now attempting landing...")
        GPIO.output(18, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(18, GPIO.LOW)

    if command == "LOOK":
        direction = capture_image(arucoDict,arucoParams, vid_stream)
        #show_image(direction, vid_stream)

    command = ""

    radio.writeAckPayload(1, ackPL, len(ackPL))
    print("Loaded payload reply of {}".format(ackPL))
    
 
