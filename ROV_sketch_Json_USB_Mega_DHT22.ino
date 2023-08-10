#include <ArduinoJson.h> //Load Json Library
#include<Servo.h> //Earlier library supported only pins 9 and 10
#include <Adafruit_Sensor.h> //Load from Tools/Manage Libraries
#include <DHT.h>
#include <Wire.h> //this library allows us to use the sda and scl ports for the PID
#include <MS5837.h>

MS5837 sensor; //sets the sensor variable to the pressure class
Servo servo_leftfront, servo_rightfront, servo_leftback,servo_rightback, servo_up1, servo_up2,servo_cam,servo_pswitch,servo_clawmanip;

int val; //variable for temperature reading
int tempPin = A1;//define analog pin to read TMP36 sensor

byte pswitch=22; //power switch
byte magnet=9;
byte DHTPIN =8; //DHT22 PWM pin assign
DHT dht = DHT(DHTPIN,DHT22);

byte servoPin_leftfront= 26; //Leftf thruster PWM
byte servoPin_rightfront= 24; // Rightf thruster PWM
byte servoPin_leftback= 34; //Leftb thruster PWM
byte servoPin_rightback= 32; // Rightb thruster PWM
byte servoPin_up1 = 28; //Up1 thruster PWM
byte servoPin_up2 = 30; //Up2 thruster PWM
byte servoPin_cam=7; //Camera 1 thruster PWM
byte servoPin_clawmanip=6; //claw servo


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);// initialize digital pin LED_BUILTIN as an output.
  digitalWrite(13,LOW);
  Serial.begin(115200);
  //Serial1.begin(9600);
  servo_up1.attach(servoPin_up1); //Attach defines servo pin as earlier library only supported pins 9 and 10
  servo_up2.attach(servoPin_up2);
  servo_leftfront.attach(servoPin_leftfront);
  servo_rightfront.attach(servoPin_rightfront);
  servo_leftback.attach(servoPin_leftback);
  servo_rightback.attach(servoPin_rightback);
  servo_cam.attach(servoPin_cam);
  servo_clawmanip.attach(servoPin_clawmanip);
  
  Wire.begin();
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)

  dht.begin(); //sensor setup
  delay(7000); //delay to allow ESC to recognize the stopped signal
}

float servoScale(float value, float minInput=-1, float maxInput=1, float minOutput=1100, float maxOutput=1900) {//better way to scale from (-1-1) to (1100-1900)
  return minOutput + (maxOutput - minOutput) * (value - minInput) / (maxInput - minInput);
}
int servoLimits(int value){
  if(value>1700){
    value=1700;
  }
  if(value<1300){
    value=1300;
  }
  return value;
}

void loop() {
  sensor.read();//reads value from Bar02 Pressure Sensor
  String thruster;
  while (!Serial.available()){ 
  //Serial.print("No data");
  digitalWrite(13,HIGH);
  delay(50);
  digitalWrite(13,LOW);
  delay(50);
  }
  if(Serial.available()) {
    thruster=Serial.readStringUntil( '\n' );//Read data from Arduino until};
    StaticJsonDocument<1000> json_doc; //the StaticJsonDocument we write to
    //DynamicJsonDocument json_doc(2048);
    //serializeJson(json_doc,Serial1);//convert to Json string,sends to surface
    //Serial1.print(thruster);
    //Serial1.println();
    deserializeJson(json_doc,thruster);
    //Power Switch
    if(json_doc["po"]==1){
      digitalWrite(pswitch,HIGH);
    }
    else{
      digitalWrite(pswitch,LOW);
    }
    //magnet
    if(json_doc["m"]==1){
      digitalWrite(magnet,HIGH);
    }
    else{
      digitalWrite(magnet,LOW);
    }
    
    //Movement thrusters
    //front left
    float fl=json_doc["fl"];
    int fl_sig=servoScale(fl);
    fl_sig=servoLimits(fl_sig);
    servo_leftfront.writeMicroseconds(fl_sig);
    
    //front right
    float fr=json_doc["fr"];
    int fr_sig=servoScale(fr);
    fr_sig=servoLimits(fr_sig);
    servo_rightfront.writeMicroseconds(fr_sig);
    
    
    //back left
    float bl=json_doc["bl"];
    int bl_sig=servoScale(bl);
    bl_sig=servoLimits(bl_sig);
    servo_leftback.writeMicroseconds(bl_sig);
    
    //back right
    float br=json_doc["br"];
    int br_sig=servoScale(br);
    br_sig=servoLimits(br_sig);
    servo_rightback.writeMicroseconds(br_sig);
    
    //Vertical Thrusters
    float th_up = json_doc["u"];
    int th_up_sig=servoScale(th_up); //map controller to servo
    int th_up_sig2=servoScale(-th_up);
    th_up_sig=servoLimits(th_up_sig);
    th_up_sig2=servoLimits(th_up_sig2);
    servo_up1.writeMicroseconds(th_up_sig); //Send signal to ESC
    servo_up2.writeMicroseconds(th_up_sig2); //Send signal to ESC

    //Camera Servo
    float cam=json_doc["c"];
    int cam_sig=servoScale(cam,-1,1,450,2450);
    servo_cam.writeMicroseconds(cam_sig);

    //Claw Manip
    float claw=json_doc["cl"];
    int claw_sig=servoScale(claw);
    claw_sig=servoLimits(claw_sig);
    servo_clawmanip.writeMicroseconds(claw_sig);

    //Read Temperature, return to surface
    //val=analogRead(tempPin);//read arduino pin
    StaticJsonDocument<1000> doc;//define StaticJsonDocument
    float mv = ((val/1024.0)*500);
    float cel = (mv/10);//temperature in Celsius

    //Read DHT22
    float h = dht.readHumidity();
    float t = dht.readTemperature(true);//Read temperature in Fahrenheit

    doc["u"]=th_up_sig;
    doc["c"]=cam_sig;
    doc["cl"]=claw_sig;
    //doc["t"]=cel;//add temp in Celsius to StaticJsonDocument
    doc["v"]=mv;// add temp in Fahrenheit 
    doc["t"]=t;//temperature from dht22
    doc["h"]=h;//humid from dht22
    doc["p"]=sensor.pressure();
    doc["d"]=sensor.depth();
    doc["a"]=sensor.altitude();
    doc["po"]=json_doc["po"];//power
    doc["m"]=json_doc["m"];//magnet
    doc["rf"]=fr_sig;
    doc["lf"]=fl_sig;
    doc["rb"]=br_sig;
    doc["lb"]=bl_sig;
    serializeJson(doc,Serial);//convert to Json string,sends to surface
    Serial.println();//newline
  }
}
    
