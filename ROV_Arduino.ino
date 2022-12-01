#include <ArduinoJson.h> //Load Json Library
#include<Servo.h> //Earlier library supported only pins 9 and 10
#include <Adafruit_Sensor.h> //Load from Tools/Manage Libraries
#include <DHT.h>

Servo servo_lff, servo_rtf, servo_lfb,servo_rtb, servo_up1, servo_up2,servo_cam1,servo_cam2;

int val; //variable for temperature reading
int tempPin = A1;//define analog pin to read TMP36 sensor

byte DHTPIN =8; //DHT22 PWM pin assign
DHT dht = DHT(DHTPIN,DHT22);

byte servoPin_rtf= 3; // Rightf thruster PWM
byte servoPin_lff= 2; //Leftf thruster PWM
byte servoPin_rtb= 6; // Rightb thruster PWM
byte servoPin_lfb= 7; //Leftb thruster PWM
byte servoPin_up1 = 5; //Up1 thruster PWM
byte servoPin_up2 = 4; //Up2 thruster PWM
byte servoPin_cam1=8; //Camera 1 thruster PWM
byte servoPin_cam2=9; //Camera 1 thruster PWM


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);// initialize digital pin LED_BUILTIN as an output.
  digitalWrite(13,LOW);
  Serial.begin(9600);
  
  servo_up1.attach(servoPin_up1); //Attach defines servo pin as earlier library only supported pins 9 and 10
  servo_up2.attach(servoPin_up2);
  servo_lff.attach(servoPin_lff);
  servo_rtf.attach(servoPin_rtf);
  servo_lfb.attach(servoPin_lfb);
  servo_rtb.attach(servoPin_rtb);
  servo_cam1.attach(servoPin_cam1);
  servo_cam2.attach(servoPin_cam2);

  dht.begin(); //sensor setup
  delay(7000); //delay to allow ESC to recognize the stopped signal
}

void loop() {
  String thruster;
  while (!Serial.available()){ 
  //Serial.print("No data");
  digitalWrite(13,HIGH);
  delay(50);
  digitalWrite(13,LOW);
  delay(50);
  }
  if(Serial.available()) {
    thruster=Serial.readStringUntil( '\x7D' );//Read data from Arduino until};
    StaticJsonDocument<1000> json_doc; //the StaticJsonDocument we write to
    deserializeJson(json_doc,thruster);
 
    //Leftf Thruster
    float front_left=json_doc["tleftf"];
    int front_left_sig=(front_left+1)*400+1100; //map controller to servo
    servo_lff.writeMicroseconds(front_left_sig); //Send signal to ESC
    
    //Rightf Thruster
    float front_right=json_doc["trightf"];
    int front_right_sig=(front_right+1)*400+1100; //map controller to servo
    servo_rtf.writeMicroseconds(front_right_sig); //Send signal to ESC
    
    //Leftb Thruster
    float back_left=json_doc["tleftb"];
    int back_left_sig=(back_left+1)*400+1100; //map controller to servo
    servo_lfb.writeMicroseconds(back_left_sig); //Send signal to ESC
    
    //Rightb Thruster
    float back_right=json_doc["trightb"];
    int back_right_sig=(back_right+1)*400+1100; //map controller to servo
    servo_rtb.writeMicroseconds(back_right_sig); //Send signal to ESC
    
    //Vertical Thruster 1 
    float th_up_1 = json_doc["tup"];
    int th_up_sig_1=(th_up_1+1)*400+1100; //map controller to servo
    servo_up1.writeMicroseconds(th_up_sig_1); //Send signal to ESC

    //Vertical Thruster 2
    float th_up_2 = json_doc["tup"];
    int th_up_sig_2=(th_up_2+1)*400+1100; //map controller to servo
    servo_up2.writeMicroseconds(th_up_sig_2); //Send signal to ESC

    //Camera 1 Servo
    float cam1=json_doc["scam1"];
    int cam_sig_1=(cam1+1)*400+1100;
    servo_cam1.writeMicroseconds(cam_sig_1);

    //Camera 2 Servo
    float cam2=json_doc["scam2"];
    int cam_sig_2=(cam2+1)*400+1100;
    servo_cam2.writeMicroseconds(cam_sig_2);

    //Read Temperature, return to surface
    val=analogRead(tempPin);//read arduino pin
    StaticJsonDocument<1000> doc;//define StaticJsonDocument
    float mv = ((val/1024.0)*500);
    float cel = (mv/10);//temperature in Celsius

    //Read DHT22
    float h = dht.readHumidity();
    float t = dht.readTemperature(true);//Read temperature in Fahrenheit

    doc["sig_up_1"]=th_up_sig_1;
    doc["sig_up_2"]=th_up_sig_2;
    doc["sig_rtf"]=front_right_sig;
    doc["sig_lff"]=front_left_sig;
    doc["sig_rtb"]=back_right_sig;
    doc["sig_lfb"]=back_left_sig;
    doc["sig_cam1"]=cam_sig_1;
    doc["sig_cam2"]=cam_sig_2;
    doc["temp"]=cel;//add temp in Celsius to StaticJsonDocument
    doc["volt"]=mv;// add temp in Fahrenheit 
    doc["temp_dht"]=t;//temperature from dht22
    doc["humid_dht"]=h;//humid from dht22




    
    serializeJson(doc,Serial);//convert to Json string,sends to surface
    Serial.println();//newline
    delay(10);
  }
}
    
