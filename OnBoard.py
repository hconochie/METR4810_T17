import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev
import random

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

    command = ""

    radio.writeAckPayload(1, ackPL, len(ackPL))
    print("Loaded payload reply of {}".format(ackPL))
    
 
