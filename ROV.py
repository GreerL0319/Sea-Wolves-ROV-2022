import json
import time
import pygame
import math  # needed for joystick
import PID
import widgets
import serial  # needed to talk with Arduino #the library needed to install is called pyserial not just serial

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
cam_display=widgets.display("Cam Servo", sidebarwidth)
manip_display=widgets.display("Manipulator",sidebarwidth)
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

def ArduinoToPython():#retrieves data from the arduino
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


    th_up_display.value = dict_json['sig_up']  # vertical thruster value from Arduino
    front_left_display.value = dict_json['left_front']  # left front thruster value from Arduino
    front_right_display.value = dict_json['right_front']  # right front thruster value from Arduino
    back_left_display.value = dict_json['left_back']  # left back thruster value from Arduino
    back_right_display.value = dict_json['right_back']  # right back thruster value from Arduino

    print(dict_json['right_front'])
    print(dict_json['right_back'])
    print(dict_json['left_back'])
    print(dict_json['right_back'])



    cam_display.value=dict_json['sig_cam'] #camera position values
    manip_display.value=dict_json['sig_claw']

    pid = p.update(dict_json['pressure']);#updates the pid with the current pressure value
    ser.flush()
    return pid

def RollingAverage(pid,pidlist):
    #here we create an array that keeps track of the last 3 instances of the pid and averages them
    pidlist.append(pid)
    pid=(pidlist[-1]+pidlist[-2]+pidlist[-3])/3 #creating an infinite array like this might use too much memory
    return pid

def PythontoArduino(commands):
    MESSAGE = json.dumps(commands)  # puts python dictionary in Json format
    ser.write(bytes(MESSAGE, 'utf-8'))  # byte format sent to arduino
    #print(ser)
    ser.flush()

def CamControl(commands):
    camaxis=0
    if joystick is not None:
        camaxis=joystick.get_axis(2)
        if abs(camaxis) < .2:  # define a dead zone
            camaxis = 0
        if onstatus.state:
            camaxis *= 1.414  # gives value of 1 for full thrust left and right
            camaxis=ServoConfinements(camaxis)
    camaxis=ServoConfinements(camaxis)
    commands["scam"]=camaxis

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
        x=float(x)
        y=float(y)
        # cube gives more control with lower power
        commands['frontleft'] = (y-crab) ** 3
        commands['frontright'] = (-x+crab) ** 3 #for some reason 45 degree angles give a value moving forward, might be the controller
        commands['backleft'] = (y+crab)**3
        commands['backright'] = (-x-crab)**3

        print(commands['frontleft'])
        print(commands['frontright'])#values are between -1 and 1 but arduino is not returning proper values???
        print(commands['backleft'])
        print(commands['backright'])


        mleftslider.value = commands['frontleft']  # assign thruster values to a display (+1,-1)
        mrightslider.value = commands['backright']
        zslider.value = commands['tup']

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

    screen.blit(cam_display.render(), (0, 5 * dheight))
    screen.blit(manip_display.render(), (220, 5 * dheight))
    screen.blit(th_up_display.render(), (0, 2 * dheight))

    #thrusters
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
vert_up=0
vert_down=0
pidtoggle=False
clawopen=False
pswitch=False
crab=0
z=0
pid=0
claw=1100
pidlist=[0,0,0]


while True:
    try:
        pid = ArduinoToPython()
    except Exception as e:
        print(e)
    pid=RollingAverage(pid,pidlist)
    pygame.event.pump()  # internally process pygame event handlers
    for event in pygame.event.get():  # get events from the queue
        if event.type == pygame.QUIT:  # clean exit on quitting (x window)
            pygame.running = False
            pygame.quit()
        elif event.type == pygame.JOYBUTTONDOWN:
            # check if a button is pushed
            if event.button == 7:
                onstatus.toggle()
            if event.button == 2:#A
                if clawopen:
                    clawopen = False
                else:
                    clawopen = True
            if event.button == 3: #X
                if pidtoggle:
                    pidtoggle=False
                else:
                    pidtoggle = True
            if event.button == 4:#LB
                crab_left = True
            if event.button == 5:#RB
                crab_right = True

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

    if pidtoggle:
        z = pid
    else:
        z=0

    if clawopen:
        claw=1900
    else:
        claw=1100
    if pidtoggle==False:
        vert_down=joystick.get_axis(4)
        vert_up=joystick.get_axis(5)
        vert_up+=1
        vert_down+=1


        if vert_up>0 and vert_down>-1:
            z=0

        if vert_up>0:#with the new controller, it seems the triggers are considered as axises instead of button presses, this is actually good because we can control our z value strength
            z=-vert_up

        elif vert_down>-1:
            z=vert_down
    z = ServoConfinements(z)
    crab = ServoConfinements(crab)
    commands = {}  # define python dictionary
    commands['tup'] = -z ** 3
    commands['sclaw']=claw
    JoystickCommands(crab, commands)  # function returns controller inputs
    CamControl(commands)
    PythontoArduino(commands)
    GuiBlit()  # blits all of our data onto the screen
