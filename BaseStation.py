import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev
import Tkinter as tk
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from time import sleep


title_font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}
axes_font = {'family' : 'normal',
        'weight' : 'normal',
        'size' : 6}

matplotlib.rc('xtick', labelsize=10)
matplotlib.rc('ytick', labelsize=10)

GPIO.setmode(GPIO.BCM)
#GPIO.setup(23, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(17, GPIO.OUT, initial=GPIO.LOW)
pipes = [[0xe7, 0xe7, 0xe7, 0xe7, 0xe7], [0xc2, 0xc2, 0xc2, 0xc2, 0xc2]]




radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)
radio.setPayloadSize(32)
radio.setChannel(0x60) #set channcel 60

radio.setDataRate(NRF24.BR_2MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()

radio.openReadingPipe(1, pipes[0])
radio.openWritingPipe(pipes[1])
radio.printDetails()

#Servo setup
GPIO.setup(2,GPIO.OUT) 
GPIO.setup(3,GPIO.OUT) 
GPIO.setup(4,GPIO.OUT)
GPIO.setup(18,GPIO.OUT) 

pwm_1 = GPIO.PWM(2,50)
pwm_2 = GPIO.PWM(3,50)
pwm_3 = GPIO.PWM(4,50)
pwm_4 = GPIO.PWM(18,50)

pwm_1.start(0)
pwm_2.start(0)
pwm_3.start(0)
pwm_4.start(0)

def land():
    """SetAngle(pwm_1,2,70)
    SetAngle(pwm_2,3,70)
    SetAngle(pwm_3,4,70)
    SetAngle(pwm_4,18,70)"""
    

    #rotate
    
    #throttle
    
    while(1):
        vis_x, vis_y = getVision()
        print(vis_x)
        print(vis_y)
        #moveX(vis_x)
        #home(4,pwm_3)
        #moveY(vis_y)
        #home(18,pwm_4)
        #loop here
        #check hunters code
        #move servos

def abort():
    GPIO.output(23, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(23, GPIO.LOW)
    
    # Send abort command
    command = "ABORT"
    message = list(command)
    radio.write(message)
    print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
        print("Our returned payload was {}".format(returnedPL))
        
def receiveData():
    print("Ready to recieve data.")
    radio.startListening()

    while not radio.available(0):
        #print("radio not available")
        time.sleep(1/100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())

    print("Translating receivedMessage into unicode characters...")
    string = ""
    for n in receivedMessage:
        if (n >= 32 and n <= 126):
                string += chr(n)
    print("Our slave sent us: {}".format(string))

    # parse message
    tempID, temp, pressureID, pressure, accelID, accel = string.split('_')
    radio.stopListening()
    return temp, pressure, accel

def receivePos():
    print("Ready to recieve data.")
    radio.startListening()

    while not radio.available(0):
        #print("radio not available")
        time.sleep(1/100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())

    print("Translating receivedMessage into unicode characters...")
    string = ""
    for n in receivedMessage:
        if (n >= 32 and n <= 126):
                string += chr(n)
    print("Our slave sent us: {}".format(string))

    # parse message
    delta_xID, delta_x, delta_yID, delta_y, delta_zID, delta_z = string.split('_')
    radio.stopListening()
    return delta_x, delta_y, delta_z

def receiveOdom():
    print("Ready to recieve data.")
    radio.startListening()

    while not radio.available(0):
        #print("radio not available")
        time.sleep(1/100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())

    print("Translating receivedMessage into unicode characters...")
    string = ""
    for n in receivedMessage:
        if (n >= 32 and n <= 126):
                string += chr(n)
    print("Our slave sent us: {}".format(string))

    # parse message
    rollID, roll, pitchID, pitch, yawID, yaw = string.split('_')
    radio.stopListening()
    return roll, pitch, yaw

def receiveVision():
    print("Ready to recieve data.")
    radio.startListening()

    while not radio.available(0):
        #print("radio not available")
        time.sleep(1/100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())

    print("Translating receivedMessage into unicode characters...")
    string = ""
    for n in receivedMessage:
        if (n >= 32 and n <= 126):
                string += chr(n)
    print("Our slave sent us: {}".format(string))

    # parse message
    a,vis_x,b, vis_y = string.split('_')
    radio.stopListening()
    return vis_x, vis_y

def getData():
    # GET DATA
    command = "GET_DATA"
    message = list(command)
    radio.write(message)
    print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
        print("Our returned payload was {}".format(returnedPL))
        temp, pressure, accel = receiveData()
    else:
        print("No Payload was received")
        temp = 0
        pressure = 0
        accel = 0
    print("temperature is: ", temp," pressure is: ", pressure, " accleration is: ", accel)
    return temp, pressure, accel


def getPos():
    # GET POSITION
    command = "GET_POS"
    message = list(command)
    radio.write(message)
    print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
        print("Our returned payload was {}".format(returnedPL))
        delta_x, delta_y, delta_z = receivePos()
    else:
        print("No Payload was received")
        delta_x = 0
        delta_y = 0
        delta_z = 0
    print("change in x: ", delta_x, " change in y: ", delta_y, " change in z: ", delta_z)
    return delta_x, delta_y, delta_z

def getOdom():
    # GET ODOMETRY
    command = "GET_ODOM"
    message = list(command)
    radio.write(message)
    print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
        print("Our returned payload was {}".format(returnedPL))
        roll, pitch, yaw = receiveOdom()
    else:
        print("No Payload was received")
        roll = 0
        pitch = 0
        yaw = 0
    print("roll: ", roll, " pitch: ", pitch, " yaw: ", yaw)
    return roll, pitch, yaw

def getVision():
    # GET ODOMETRY
    command = "LOOK"
    message = list(command)
    radio.write(message)
    print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
        print("Our returned payload was {}".format(returnedPL))
        vis_x, vis_y = receiveVision()
    else:
        print("No Payload was received")
        vis_x = 0
        vis_y = 0
    print("vis_x: ", vis_x, " vis_y: ", vis_y)
    return vis_x, vis_y

#SERVO FUNCTIONS
def SetAngle(pwm,pin,angle):
    duty_cycle = angle/18 +2
    GPIO.output(pin,True)
    pwm.ChangeDutyCycle(duty_cycle)
    sleep(1)
    GPIO.output(pin,False)
    pwm.ChangeDutyCycle(0)
    
def moveX(transX):
    #X and Y are highest pixel counts given by hunters code
    if 0.1 <= transX <= 1 :
        angle = 70+transX*50
        duty_cycle = angle/18 + 2
        GPIO.output(4,True)
        pwm_3.ChangeDutyCycle(duty_cycle)
        sleep(0.5)
        GPIO.output(4,False)
        pwm_3.ChangeDutyCycle(0)
    

def moveY(transY):
    #X and Y are highest pixel counts given by hunters code
    if 0.1 <= transY <= 1:
        angle = 70+transY*50
        duty_cycle = angle/18 + 2
        GPIO.output(18,True)
        pwm_3.ChangeDutyCycle(duty_cycle)
        sleep(0.5)
        GPIO.output(18,False)
        pwm_3.ChangeDutyCycle(0)

def home(pin,pwm):
    duty_cycle = 90/18 +2
    GPIO.output(pin,True)
    pwm.ChangeDutyCycle(duty_cycle)
    sleep(1)
    GPIO.output(pin,False)
    pwm.ChangeDutyCycle(0)

# ============ GUI =========
root = tk.Tk()
root.title('METR4810 Mission Control GUI')
root.config(background='#fafafa')

xar = []
yar = []
yar2 = []
yar3 = []
yar4 = []
yar5 = []
yar6 = []
yar7 = []
yar8 = []
yar9 = []

style.use('ggplot')
print(plt.style.available)
fig = plt.figure(figsize=(8, 16), dpi=80)
ax1 = fig.add_subplot(5,1,1)
ax1.set_ylim(0, 50)
line1, = ax1.plot(xar, yar, 'r', marker='o')
#ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Temperature (C)', **axes_font)
ax1.set_title('Temperature Plot', **title_font)

ax2 = fig.add_subplot(5,1,2)
ax2.set_ylim(0, 4)
line2, = ax2.plot(xar, yar2, 'r', marker='o')
ax2.set_ylabel('Pressure (MPa)', **axes_font)
ax2.set_title('Pressure Plot', **title_font)

ax3 = fig.add_subplot(5,1,3)
ax3.set_ylim(0, 2)
line3, = ax3.plot(xar, yar3, 'r', marker='o')
ax3.set_ylabel("Acceleration (m/s^2)", **axes_font)
ax3.set_title("Acceleration Plot", **title_font)


ax4 = fig.add_subplot(5,1,4)
ax4.set_ylim(0, 3)
line4, = ax4.plot(xar, yar4, 'r', marker='o')
line5, = ax4.plot(xar, yar5, 'g', marker='o')
line6, = ax4.plot(xar, yar6, 'b', marker='o')
ax4.set_ylabel("Change in position from launch (m)", **axes_font)
ax4.set_title("Position Plot", **title_font)
ax4.legend("xyz", loc="upper left")

ax5 = fig.add_subplot(5,1,5)
ax5.set_ylim(0, 360)
line7, = ax5.plot(xar, yar7, 'r', marker='o')
line8, = ax5.plot(xar, yar8, 'g', marker='o')
line9, = ax5.plot(xar, yar9, 'b', marker='o')
ax5.set_ylabel("Orientation (degrees from neurtral)", **axes_font)
ax5.set_title("Orientation Plot", **title_font)
ax5.legend("rpy", loc="upper left")

plt.tight_layout(pad=3.0)

def animate(i):
    temp, pressure, accel = getData()
    delta_x, delta_y, delta_z = getPos()
    roll, pitch, yaw = getOdom()
    # append temperature
    yar.append(temp)
    yar2.append(pressure)
    yar3.append(accel)
    yar4.append(delta_x)
    yar5.append(delta_y)
    yar6.append(delta_z)
    yar7.append(roll)
    yar8.append(pitch)
    yar9.append(yaw)
    xar.append(i)
    line1.set_data(xar, yar)
    line2.set_data(xar, yar2)
    line3.set_data(xar, yar3)
    line4.set_data(xar, yar4)
    line5.set_data(xar, yar5)
    line6.set_data(xar, yar6)
    line7.set_data(xar, yar7)
    line8.set_data(xar, yar8)
    line9.set_data(xar, yar9)
    ax1.set_xlim(0, i+1)
    ax2.set_xlim(0, i+1)
    ax3.set_xlim(0, i+1)
    ax4.set_xlim(0, i+1)
    ax5.set_xlim(0, i+1)

topFrame = tk.Frame(root)
topFrame.pack(side = tk.TOP)

landButton = tk.Button(topFrame, text="Landing Sequence", command=land)
abortButton = tk.Button(topFrame, text='ABORT!', command= abort)
plotcanvas = FigureCanvasTkAgg(fig, root)

landButton.pack(side = tk.LEFT)
abortButton.pack(side= tk.RIGHT)
plotcanvas.get_tk_widget().pack()
#ani = animation.FuncAnimation(fig, animate, interval=1000, blit=False)
root.mainloop()

pwm_1.stop()
pwm_2.stop()
pwm_3.stop()
pwm_4.stop()
GPIO.cleanup()
