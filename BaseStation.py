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


title_font = {'family' : 'DejaVu Sans',
        'weight' : 'bold',
        'size'   : 12}
axes_font = {'family' : 'DejaVu Sans',
        'weight' : 'bold',
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
radio.setPALevel(NRF24.PA_MAX)
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

#global check = 0 #Check value for if the craft has dropped.
def drop():
    # Trust up
    
    SetAngle(pwm_1,2,80)
    sleep(1)
    SetAngle(pwm_1,2,70)
    sleep(1)
    SetAngle(pwm_1,2,60)
    sleep(0.2)
    SetAngle(pwm_1,2,50)
    
    sleep(2)
    SetAngle(pwm_2,3,77)
    sleep(1.5)
    SetAngle(pwm_2,3,90)
    sleep(0.5)
    SetAngle(pwm_2,3,90)
    sleep(0.5)
    SetAngle(pwm_2,3,90)
    #check = 1
def rotate():
    SetAngle(pwm_2,3,75)
    sleep(1)
    SetAngle(pwm_2,3,90)
    sleep(0.2)
    
land_sleep = 0.01
def land():

    x_v,y_v = getVision()
    #print("xv",x_v,"yv",y_v)

    if x_v == -1:
        abort()
    if 0 < x_v < 0.5:
        SetAngle(pwm_3,4,75)
        sleep(land_sleep)
        SetAngle(pwm_3,4,90)
        SetAngle(pwm_3,4,90)
        
    elif 1 > x_v > 0.5:
        SetAngle(pwm_3,4,115)
        sleep(land_sleep)
        SetAngle(pwm_3,4,90)
        SetAngle(pwm_3,4,90)
        
    sleep(land_sleep)

    if 0 < y_v < 0.5:
        SetAngle(pwm_4,18,75)
        sleep(land_sleep)
        SetAngle(pwm_4,18,90)
        SetAngle(pwm_4,18,90)
    elif 1 > y_v > 0.5:
        SetAngle(pwm_4,18,115)
        sleep(land_sleep)
        SetAngle(pwm_4,18,90)
        SetAngle(pwm_4,18,90)


    #print(anglex)
    #SetAngle(pwm_3,4,anglex)
    #sleep(0.1)
    #SetAngle(pwm_3,4,90)
        
def abort():
    SetAngle(pwm_1,2,120)

def receiveData():
    #print("Ready to recieve data.")
    radio.startListening()

    while not radio.available(0):
        #print("radio not available")
        time.sleep(1/100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())

    #print("Translating receivedMessage into unicode characters...")
    string = ""
    for n in receivedMessage:
        if (n >= 32 and n <= 126):
                string += chr(n)
    #print("Our slave sent us: {}".format(string))

    # parse message
    tempID, temp, pressureID, pressure = string.split('_')
    radio.stopListening()
    return temp, pressure

def receiveAcc():
    #print("Ready to recieve data.")
    radio.startListening()

    while not radio.available(0):
        #print("radio not available")
        time.sleep(1/100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())

    #print("Translating receivedMessage into unicode characters...")
    string = ""
    for n in receivedMessage:
        if (n >= 32 and n <= 126):
                string += chr(n)
    #print("Our slave sent us: {}".format(string))

    # parse message
    axID, ax, ayID, ay, azID, az = string.split('_')
    radio.stopListening()
    return ax, ay, az



def receiveOdom():
    #print("Ready to recieve data.")
    radio.startListening()

    while not radio.available(0):
        #print("radio not available")
        time.sleep(1/100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())

    #print("Translating receivedMessage into unicode characters...")
    string = ""
    for n in receivedMessage:
        if (n >= 32 and n <= 126):
                string += chr(n)
    #print("Our slave sent us: {}".format(string))

    # parse message
    rollID, roll, pitchID, pitch, yawID, yaw = string.split('_')
    radio.stopListening()
    return roll, pitch, yaw

def receiveVision():
    #print("Ready to recieve data.")
    radio.startListening()

    while not radio.available(0):
        #print("radio not available")
        time.sleep(1/100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())

    #print("Translating receivedMessage into unicode characters...")
    string = ""
    for n in receivedMessage:
        if (n >= 32 and n <= 126):
                string += chr(n)
    #print("Our slave sent us: {}".format(string))

    # parse message
    a,vis_x,b, vis_y = string.split('_')
    
    radio.stopListening()
    return vis_x, vis_y

def getData():
    # GET DATA
    command = "GET_DATA"
    message = list(command) 
    radio.write(message)
    #print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
        #print("Our returned payload was {}".format(returnedPL))
        temp, pressure= receiveData()
    else:
        #print("No Payload was received")
        temp = 0
        pressure = 0
    #print("temperature is: ", temp," pressure is: ", pressure)
    return float(temp), float(pressure)



def getAcc():
    # GET ACCELERATION
    command = "GET_ACC"
    message = list(command)
    radio.write(message)
    #print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
        #print("Our returned payload was {}".format(returnedPL))
        ax,ay,az = receiveAcc()
    else:
        #print("No Payload was received")
        ax = 0
        ay = 0
        az = 0
    #print("aX: ", ax," aY: ", ay, " aZ: ", az)
    return float(ax),float(ay),float(az)



def getOdom():
    # GET ODOMETRY
    command = "GET_ODOM"
    message = list(command)
    radio.write(message)
    #print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
        #print("Our returned payload was {}".format(returnedPL))
        roll, pitch, yaw = receiveOdom()
    else:
        #print("No Payload was received")
        roll = 0
        pitch = 0
        yaw = 0
    #print("roll: ", roll, " pitch: ", pitch, " yaw: ", yaw)
    return float(roll), float(pitch), float(yaw)

def getVision():
    # GET ODOMETRY
    command = "LOOK"
    message = list(command)
    radio.write(message)
    #print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
        #print("Our returned payload was {}".format(returnedPL))
        vision_x, vision_y = receiveVision()
        vis_x = float(vision_x)
        vis_y = float(vision_y)
    else:
        #print("No Payload was received")
        vis_x = 0.5
        vis_y = 0.5
    print("vis_x: ", vis_x, " vis_y: ", vis_y)
    return vis_x, vis_y


def sendPowerCycle():
    # GET DATA
    command = "POWER_CYCLE"
    message = list(command) 
    radio.write(message)
    #print("We sent the message of {}".format(message))
    # check for ack payload
    if radio.isAckPayloadAvailable():
        returnedPL = []
        returnedPL = radio.read(returnedPL, radio.getDynamicPayloadSize())
    else:
        print("No Payload was received - PowerCycle")
        

        
        
#SERVO FUNCTIONS
def SetAngle(pwm,pin,angle):
    duty_cycle = angle/18 +2
    pwm.ChangeDutyCycle(duty_cycle)
    sleep(0.1)
    pwm.ChangeDutyCycle(0)


    
#alt_init = []
#def landed(ax,ay,az):
    
#    if  :
#        return True
#    else:
#        return False



    
    
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

style.use('ggplot')
# print(plt.style.available)
fig = plt.figure(figsize=(8, 16), dpi=80)
ax1 = fig.add_subplot(4,1,1)
ax1.set_ylim(0, 50)
line1, = ax1.plot(xar, yar, 'r', marker='o')
#ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Temperature (C)', **axes_font)
ax1.set_title('Temperature Plot', **title_font)

ax2 = fig.add_subplot(4,1,2)
ax2.set_ylim(0, 200)
line2, = ax2.plot(xar, yar2, 'r', marker='o')
ax2.set_ylabel('Pressure (MPa)', **axes_font)
ax2.set_title('Pressure Plot', **title_font)

ax3 = fig.add_subplot(4,1,3)
ax3.set_ylim(-1, 1)
line3, = ax3.plot(xar, yar3, 'r', marker='o')
line4, = ax3.plot(xar, yar4, 'g', marker='o')
line5, = ax3.plot(xar, yar5, 'b', marker='o')
ax3.set_ylabel("Acceleration (m/s^2)", **axes_font)
ax3.set_title("Acceleration Plot", **title_font)
ax3.legend("xyz", loc="upper left")



ax4 = fig.add_subplot(4,1,4)
ax4.set_ylim(-180, 180)
line6, = ax4.plot(xar, yar6, 'r', marker='o')
line7, = ax4.plot(xar, yar7, 'g', marker='o')
line8, = ax4.plot(xar, yar8, 'b', marker='o')
ax4.set_ylabel("Change in odometry from launch (deg)", **axes_font)
ax4.set_title("Odometry Plot", **title_font)
ax4.legend("rpy", loc="upper left")


plt.tight_layout(pad=3.0)


def animate(i):
    
    #if not landed(ax,ay,az) and check == 1:
    land()
    #else:
        #alt_init.append(getAltitude)
    #temp, pressure= getData()
    temp, pressure = (0,0)
    #ax,ay,az = getAcc()
    #print("accel values x, y, z: ", ax, " ", ay, " ", az)
    ax,ay,az = (0,0,0)
    #roll, pitch, yaw = getOdom()
    roll,pitch,yaw = (0,0,0)
    # append temperature

    yar.append(temp)
    yar2.append(pressure)
    yar3.append(ax)
    #print(yar3)
    yar4.append(ay)
    #print(yar4)
    yar5.append(az)
    #print(yar5)

    yar6.append(roll)
    yar7.append(pitch)
    yar8.append(yaw)
    
    xar.append(i)
    line1.set_data(xar, yar)
    line2.set_data(xar, yar2)
    line3.set_data(xar, yar3)
    line4.set_data(xar, yar4)
    line5.set_data(xar, yar5)
    line6.set_data(xar, yar6)
    line7.set_data(xar, yar7)
    line8.set_data(xar, yar8)

    ax1.set_xlim(0, i+0.8)
    ax2.set_xlim(0, i+0.8)
    ax3.set_xlim(0, i+0.8)
    ax4.set_xlim(0, i+0.8)
    
    


topFrame = tk.Frame(root)
topFrame.pack(side = tk.TOP)

landButton = tk.Button(topFrame, text="Landing Sequence", command=drop)
rotateButton= tk.Button(topFrame, text = "ROTATE", command = rotate)
abortButton = tk.Button(topFrame, text='ABORT!', command= abort)
powerButton = tk.Button(topFrame, text='PowerCycle', command= sendPowerCycle)
plotcanvas = FigureCanvasTkAgg(fig, root)

landButton.pack(side = tk.LEFT)
rotateButton.pack(side = tk.LEFT)
abortButton.pack(side= tk.RIGHT)
powerButton.pack(side = tk.RIGHT)
plotcanvas.get_tk_widget().pack()
ani = animation.FuncAnimation(fig, animate, interval=400, blit=False)

root.mainloop()

pwm_1.stop()  
pwm_2.stop()
pwm_3.stop()
pwm_4.stop()
GPIO.cleanup()
