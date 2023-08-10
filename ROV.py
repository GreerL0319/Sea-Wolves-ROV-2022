import json
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
magnet_display=widgets.display("Magnet",sidebarwidth)

#Servos
th_up_display = widgets.display("Servo Up", sidebarwidth)
cam_display=widgets.display("Cam Servo", sidebarwidth)
manip_display=widgets.display("Manipulator",sidebarwidth)
power_display=widgets.display("Power",sidebarwidth)
#Thrusters
front_left_display = widgets.display("FL Thruster", sidebarwidth)
front_right_display = widgets.display("FR Thruster", sidebarwidth)
back_left_display = widgets.display("BL Thruster", sidebarwidth)
back_right_display = widgets.display("BR Thruster", sidebarwidth)

# open serial com to Arduino
ser = serial.Serial(port='COM5', baudrate=115200, timeout=.1,dsrdtr=True)  # dsrdtr=True stops Arduino Mega from autoresetting

#pid setup; the tunings will probably need to change for our robot
#PID(GAIN,RESET,PREACT)
p=PID.PID(.1,.05,.1)
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
    #print(data)
    dict_json = json.loads(data)  # data from arduino in dictionary form

    dict_json['p']=round(dict_json["p"],2)#pressure
    dict_json["a"] = round(dict_json["a"],2)#altitude
    dict_json["d"] = round(dict_json["d"],2)#depth


    #print(dict_json)
    temp_dht22_display.value = dict_json['t']  # temp from DHT22 sensor
    humid_dht22_display.value = dict_json['h']  # humid from DHT sensor
    temp_display.value = dict_json['v']  # assign temp in Fahrenheit to display
    pressure_display.value = dict_json['p']
    altitude_display.value = dict_json['a']
    depth_display.value = dict_json['d']


    th_up_display.value = dict_json['u']  # vertical thruster value from Arduino
    front_left_display.value = dict_json['lf']  # left front thruster value from Arduino
    front_right_display.value = dict_json['rf']  # right front thruster value from Arduino
    back_left_display.value = dict_json['lb']  # left back thruster value from Arduino
    back_right_display.value = dict_json['rb']  # right back thruster value from Arduino

    cam_display.value=dict_json['c'] #camera position values
    manip_display.value=dict_json['cl']#claw
    if dict_json['po']==1:
        power_display.value="ON"
    else:
        power_display.value="OFF"
    if dict_json['m']==1:
        magnet_display.value="ON"
    else:
        magnet_display.value="OFF"

    pid = p.update(dict_json['p']);#updates the pid with the current pressure value
    ser.flush()
    return pid

def PythontoArduino(commands):
    
    MESSAGE = json.dumps(commands)  # puts python dictionary in Json format
    MESSAGE=MESSAGE+"\n"
    ser.write(bytes(MESSAGE, 'utf-8'))  # byte format sent to arduino
    #print(ser)
    ser.flush()

def RollingAverage(value,vallist):
    #here we create an array that keeps track of the last 3 instances of the pid and averages them
    vallist.append(value)
    value=(vallist[-1]+vallist[-2]+vallist[-3]+vallist[-4]+vallist[-5]+vallist[-6])/6
    vallist.pop(0)#removes the first item
    return value



def CamControl(commands,camvalue,camaxis):
    if joystick is not None:
        camaxis=joystick.get_axis(3)
        if abs(camaxis) < .2:  # define a dead zone\
            camaxis = 0
        if camaxis>=.3:
            camvalue+=.05
        if camaxis<=-.3:
            camvalue-=.05
        camvalue=ServoConfinements(camvalue)
        camvalue=round(camvalue,2)
    if onstatus.state:
        commands["c"]=camvalue
    return camvalue

def JoystickCommands(crab,commands,xlist,ylist):
    limit=.75
    x1=0
    y1=0
    if crab==0:
        if joystick is not None:
            x1 = joystick.get_axis(2)  # left joystick -1 is left to +1 is right (left thruster)
            y1 = joystick.get_axis(1)  # left joystick -1 is up +1 is down (right thruster)
    #since deadzone is between -.2 and .2 we need it to start scaling after
    if x1>0:
        x1-=.2
    if x1<0:
        x1+=.2
    if y1>0:
        y1-=.2
    if y1<0:
        y1+=.2
    x1*=(1.414*limit)
    y1*=(1.414*limit)
    if abs(y1) < .2:  # define a dead zone
        y1 = 0
    if abs(x1) < .2:  # define a dead zone
        x1 = 0
    if onstatus.state:
        # rotate x and y axis of joystick 45 degrees

        x = (x1 * math.cos(math.pi / 4))-(y1 * math.sin(math.pi / 4))  # horizontal left
        y = (x1 * math.sin(math.pi / 4)) + (y1 * math.cos(math.pi / 4))  # horizontal right
        # limits joystick values to +/- 1
        x=ServoConfinements(x)
        y=ServoConfinements(y)

        x=RollingAverage(x,xlist)
        y=RollingAverage(y,ylist)
        # add to dictionary
        # cube gives more control with lower power
        commands['fl'] = round(((y+crab)**3),2)
        commands['fr'] = round(((x+crab)**3),2) #for some reason 45 degree angles give a value moving forward, might be the controller
        commands['bl'] = round(((-y+crab)**3),2)
        commands['br'] = round(((-x+crab)**3),2)

        """
        print(commands['frontleft'])
        print(commands['frontright'])#values are between -1 and 1 but arduino is not returning proper values???
        print(commands['backleft'])
        print(commands['backright'])
        """

        mleftslider.value = commands['fl']  # assign thruster values to a display (+1,-1)
        mrightslider.value = commands['br']
        zslider.value = commands['u']

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
    screen.blit(power_display.render(),(0,6*dheight))
    screen.blit(magnet_display.render(),(220,6*dheight))

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
magnettoggle=False
crab_right = False
claw_close=False
claw_open=False
vert_up=0
vert_down=0
pidtoggle=False
pswitch=False
crab=0
z=0
pid=0
claw=0
camvalue=0
camaxis=0
#lists used to calculate averages
pidlist=[0,0,0,0,0,0]
xlist=[0,0,0,0,0,0]
ylist=[0,0,0,0,0,0]
zlist=[0,0,0,0,0,0,]
powertoggle=False
zlimit=.5
while True:
    try:
        pid = ArduinoToPython()
    except Exception as e:
        print(e)
    RollingAverage(pid,pidlist)
    pygame.event.pump()  # internally process pygame event handlers
    for event in pygame.event.get():  # get events from the queue
        if event.type == pygame.QUIT:  # clean exit on quitting (x window)
            pygame.running = False
            pygame.quit()
        elif event.type == pygame.JOYBUTTONDOWN:
            # check if a button is pushed
            if event.button == 7:
                onstatus.toggle()
                if powertoggle:
                    powertoggle=False
                else:
                    powertoggle = True
            if event.button==0:
                if magnettoggle:
                    magnettoggle=False
                else:
                    magnettoggle=True
            if event.button == 1:#B
                claw_close=True
            if event.button==2:
                claw_open=True
            if event.button == 3: #Y
                if pidtoggle:
                    pidtoggle=False
                else:
                    pidtoggle = True
            if event.button == 4:#LB
                crab_left = True
            if event.button == 5:#RB
                crab_right = True

        elif event.type == pygame.JOYBUTTONUP:  # this format with the TrueFalse values and joybutton up seems to be the only way pygame can check if a button is held
            if event.button==1:
                claw_close=False
                claw=0
            if event.button==2:
                claw_open=False
                claw=0
            if event.button == 4:
                crab_left = False
                crab = 0
            if event.button == 5:
                crab_right = False
                crab = 0

    if crab_left:
        crab += .1
    if crab_right:
        crab -= .1

    if claw_open:
        claw+=.1
    if claw_close:
        claw-=.1
    if pidtoggle:
        pressure=dict_json["p"]
        z = pid
    else:
 
        z=0
    if pidtoggle==False:
        vert_down=joystick.get_axis(4)
        vert_up=joystick.get_axis(5)
        vert_up+=1
        vert_down+=1#gets rid of negative indexes on each trigger
        if vert_up>0 and vert_down>-1:
            z=0

        if vert_up>0:#with the new controller, it seems the triggers are considered as axises instead of button presses, this is actually good because we can control our z value strength
            z=-vert_up

        elif vert_down>-1:
            z=vert_down
    z*=zlimit
    z = ServoConfinements(z)
    z=float(z)
    z=RollingAverage(z,zlist)
    crab = ServoConfinements(crab)
    commands = {}  # define python dictionary
    if powertoggle:
        commands['po']=1
    else:
        commands['po']=0
    if magnettoggle:
        commands['m']=1
    else:
        commands['m']=0
    commands['u'] = -z ** 3
    commands['cl']=claw
    JoystickCommands(crab, commands,xlist,ylist)  # function returns controller inputs
    camvalue=CamControl(commands,camvalue,camaxis)
    PythontoArduino(commands)
    GuiBlit()  # blits all of our data onto the screen
