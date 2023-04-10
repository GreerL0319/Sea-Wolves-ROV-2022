#import sys
import json
import time
import pygame
import math  # needed for joystick
import PID
#from pygame.rect import Rect
import widgets
import serial  # needed to talk with Arduino

# GUI window setup
sidebarwidth = 220
pygame.init()
pygame.display.set_caption('ROV Control')
#icon=pygame.image.load("icon.png")
#pygame.display.set_icon(icon)
size = width, height = 220 + sidebarwidth, 520  # size of GUI

# setup displays in GUI
screen = pygame.display.set_mode(size)
onstatus = widgets.toggleable("Running", sidebarwidth)  # label and size toggle
zslider = widgets.sliderdisplay("Z", 75, 160)
mleftslider = widgets.sliderdisplay("Leftslider", 75, 160)
mrightslider = widgets.sliderdisplay("Rightslider", 75, 160)
temp_display = widgets.display("Temp_TMP36 (F)", sidebarwidth)  # tmp36 sensor
pressure_display = widgets.display("Pressure (Pa)", sidebarwidth)  # tmp36 sensor
temp_dht22_display = widgets.display("Temp_DHT22 (F)", sidebarwidth)  # DHT22 sensor
humid_dht22_display = widgets.display("Humidity (g/kg)", sidebarwidth)  # DHT22 sensor
altitude_display=widgets.display("Altitude (m)",sidebarwidth)
depth_display=widgets.display("Depth (m)",sidebarwidth)

#Servos
th_up_display = widgets.display("Servo Up", sidebarwidth)
cam1_display=widgets.display("Cam 1 Servo", sidebarwidth)
cam2_display=widgets.display("Cam 2 Servo", sidebarwidth)

#Thrusters
front_left_display = widgets.display("FL Thruster", sidebarwidth)
front_right_display = widgets.display("FR Thruster", sidebarwidth)
back_left_display = widgets.display("BL Thruster", sidebarwidth)
back_right_display = widgets.display("BR Thruster", sidebarwidth)

# open serial com to Arduino
ser = serial.Serial(port='COM5', baudrate=9600, timeout=.1,dsrdtr=True)  # dsrdtr=True stops Arduino Mega from autoresetting

#pid setup; the tunings will probably need to change for our robot
#PID(GAIN,RESET,PREACT)
p=PID.PID(.01,.004,.01)
p.setPoint(0)

# init joystick
joystick = None
if pygame.joystick.get_count() == 0:
    print('No joystick Detected')
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()  # initalize joystick

def ServoConfinements(servo):#servos need to stay between value of 1 and -1 (1100 or 1900)
    if servo>1:
        servo=1
    if servo<-1:
        servo=-1
    return servo

def ArduinoToPython(mvgavg):#retrieves data from the arduino
    data = ser.readline().decode("utf-8")  # decode into byte from Arduino
    print(data)
    dict_json = json.loads(data)  # data from arduino in dictionary form

    dict_json['pressure']=round(dict_json['pressure'])
    dict_json["altitude"] = round(dict_json["altitude"],2)
    dict_json["depth"] = round(dict_json["depth"],2)


    # print(dict_json)
    temp_dht22_display.value = dict_json['temp_dht']  # temp from DHT22 sensor
    humid_dht22_display.value = dict_json['humid_dht']  # humid from DHT sensor
    temp_display.value = dict_json['volt']  # assign temp in Fahrenheit to display
    pressure_display.value = dict_json['pressure']
    altitude_display.value = dict_json['altitude']
    depth_display.value = dict_json['depth']


    th_up_display.value = dict_json['sig_up_1']  # vertical thruster value from Arduino
    front_left_display.value = dict_json['sig_lff']  # left front thruster value from Arduino
    front_right_display.value = dict_json['sig_rtf']  # right front thruster value from Arduino
    back_left_display.value = dict_json['sig_lfb']  # left back thruster value from Arduino
    back_right_display.value = dict_json['sig_rtb']  # right back thruster value from Arduino
    cam1_display.value=dict_json['sig_cam1'] #camera position values
    cam2_display.value=dict_json['sig_cam2']

    pid = p.update(dict_json['pressure']);
    ser.flush()
    return pid

def RollingAverage(pid,pidlist):
    #here we create an array that keeps track of the last 3 instances of the pid and averages them
    pidlist.append(pid)
    pid=(mvgavg(-1)+mvgavg(-2)+mvgavg(-3))/3 #creating an infinite array like this might use too much memory
    return pid

def PythontoArduino(commands):
    MESSAGE = json.dumps(commands)  # puts python dictionary in Json format
    ser.write(bytes(MESSAGE, 'utf-8'))  # byte format sent to arduino
    #print(ser)
    ser.flush()

def CamControl(commands,camstate):
    camaxis=0
    if joystick is not None:
        camaxis=joystick.get_axis(2)
        if abs(camaxis) < .2:  # define a dead zone
            camaxis = 0
        if onstatus.state:
            camaxis *= 1.414  # gives value of 1 for full thrust left and right
            camaxis=ServoConfinements(camaxis)
    if camstate==0:
        commands["scam1"]=camaxis
        commands["scam2"]=0
    elif camstate==1:
        commands["scam1"]=0
        commands["scam2"]=camaxis

def JoystickCommands(crab,commands):
    x=0
    y=0
    if crab==0:
        if joystick is not None:
            x = joystick.get_axis(0)  # left joystick -1 is left to +1 is right (left thruster)
            y = joystick.get_axis(1)  # left joystick -1 is up +1 is down (right thruster)
    if abs(y) < .2:  # define a dead zone
        y = 0
    if abs(x) < .2:  # define a dead zone
        x = 0
    if onstatus.state:
        y *= -1.414  # gives value of 1 for full thrust forward and backwards
        x *= 1.414  # gives value of 1 for full thrust forward and backwards

        # rotate x and y axis of joystick 45 degrees
        x = (x * math.cos(math.pi / 4))-(y * math.sin(math.pi / 4))  # horizontal left
        y = (x * math.sin(math.pi / -4)) + (y * math.cos(math.pi / -4))  # horizontal right
        # limits joystick values to +/- 1
        x=ServoConfinements(x)
        y=ServoConfinements(y)
        # add to dictionary
        # cube gives more control with lower power
        commands['tleftf'] = (y-crab) ** 3
        commands['trightf'] = (-x+crab) ** 3 #for some reason 45 degree angles give a value moving forward, might be the controller
        commands['tleftb'] = (y+crab)**3
        commands['trightb'] = (-x-crab)**3
        mleftslider.value = commands['tleftf']  # assign thruster values to a display (+1,-1)
        mrightslider.value = commands['trightf']
        zslider.value = commands['tup']
        return commands

def GuiBlit():
    dheight = onstatus.get_height()
    screen.blit(onstatus.render(), (0, 0))
    screen.blit(mleftslider.render(), (0, 15 * dheight))
    screen.blit(mrightslider.render(), (73, 15 * dheight))
    screen.blit(zslider.render(), (146, 15 * dheight))  # blitting thruster values
    screen.blit(temp_display.render(),
                (0, dheight))  # blitting temperature values pick a font you have and set its size
    screen.blit(temp_dht22_display.render(), (220, dheight))
    screen.blit(humid_dht22_display.render(), (220, 2 * dheight))
    screen.blit(pressure_display.render(), (0, 6 * dheight))
    screen.blit(altitude_display.render(), (220, 6 * dheight))
    screen.blit(depth_display.render(), (0, 7 * dheight))

    screen.blit(cam1_display.render(), (0, 5 * dheight))
    screen.blit(cam2_display.render(), (220, 5 * dheight))
    screen.blit(th_up_display.render(), (0, 2 * dheight))

    screen.blit(front_left_display.render(), (0, 3 * dheight))
    screen.blit(front_right_display.render(), (220, 3 * dheight))
    screen.blit(back_left_display.render(), (0, 4 * dheight))
    screen.blit(back_right_display.render(), (220, 4 * dheight))

    # Label GUI
    myfont = pygame.font.SysFont("cambriacambriamath", 14)  # define font
    th_up = myfont.render("Z", 0, (230, 0, 0), (230, 230, 0))  # render(text, antialias,color,background)
    screen.blit(th_up, (170, 14 * dheight))  # put the label object on the screen
    front_left = myfont.render("front_left", 1, (230, 0, 0), (230, 230, 0))
    screen.blit(front_left, (0, 14 * dheight))  # put the label object on the screen
    front_right = myfont.render("front_right", 1, (230, 0, 0), (230, 230, 0))
    screen.blit(front_right, (0, 14 * dheight))  # put the label object on the screen

    pygame.display.flip()  # update screen
    time.sleep(0.01)

crab_left = False
crab_right = False
camtoggle=False
vert_up=False
vert_down=False
pidtoggle=False
crab=0
z=0
pid=0
pidlist()
while True:
    try:
        pid = ArduinoToPython()
    except Exception as e:
        print(e)
    pid=RollingAverage(pid,pidlist)
    camstate = 0
    pygame.event.pump()  # internally process pygame event handlers
    for event in pygame.event.get():  # get events from the queue
        if event.type == pygame.QUIT:  # clean exit on quitting (x window)
            pygame.exit()
            pygame.running = False
        elif event.type == pygame.JOYBUTTONDOWN:
            # check if a button is pushed
            if event.button == 9:  # The button with 1 is pushed
                onstatus.toggle()
            if event.button == 2:
                if pidtoggle:
                    pidtoggle=False
                else:
                    pidtoggle = True
            if event.button == 3:
                if camtoggle:
                    camtoggle = False
                else:
                    camtoggle = True
            if event.button == 4:
                crab_left = True
            if event.button == 5:
                crab_right = True
            if event.button == 6:
                vert_up = True
            if event.button == 7:
                vert_down = True

        elif event.type == pygame.JOYBUTTONUP:  # this format with the TrueFalse values and joybutton up seems to be the only way pygame can check if a button is held
            if event.button == 4:
                crab_left = False
                crab = 0
            if event.button == 5:
                crab_right = False
                crab = 0
            if event.button == 6:
                vert_up = False
                z = 0
            if event.button == 7:
                vert_down = False
                z = 0
    if crab_left:
        crab -= .1
    if crab_right:
        crab += .1
    if camtoggle:
        camstate = 1
    if vert_up:
        z += .1
    if vert_down:
        z -= .1
    print(pid)
    print(pidtoggle)
    if pidtoggle:
        z = pid;
    print(z)
    z = ServoConfinements(z)
    crab = ServoConfinements(crab)
    commands = {}  # define python dictionary
    commands['tup'] = -z ** 3
    JoystickCommands(crab, commands)  # function returns controller inputs
    CamControl(commands, camstate)
    PythontoArduino(commands)
    GuiBlit()  # blits all of our data onto the screen

