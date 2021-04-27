#Imports to make vision system work
from imutils.video.pivideostream import PiVideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import imutils
import time
import cv2
import sys


#Print data for controlling timing
print_to_terminal = False

#requires print_to_terminal
print_time_of_function = True


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


	


arucoDict, arucoParams, vid_stream = initialize_camera()
time.sleep(2)
start_time = time.perf_counter()


i = 1
total_frames = 100000
while i <= total_frames:
	direction = capture_image(arucoDict,arucoParams, vid_stream)
	print(direction)
	
	#show_image(direction, vid_stream)
	
	i += 1
	
vid_stream.stop()

if print_time_of_function:
	finish_time = time.perf_counter()
	#print(f"Total time = {finish_time - start_time:0.4f} seconds")
	print(f"FPS= {total_frames/(finish_time - start_time):0.4f} Frames per Second")

