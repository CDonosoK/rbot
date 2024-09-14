#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>

// CONTROLLER INPUTS VARIABLES
#define CH1 12 // SWD
#define CH2 14 // SWC
#define CH3 25 // LEFT Y AXIS
#define CH4 33 // LEFT X AXIS
#define CH5 26 // RIGHT Y AXIS
#define CH6 27 // RIGHT X AXIS
int valueCH1, valueCH2, valueCH3, valueCH4, valueCH5, valueCH6;

// MOTORS VARIABLES
#define motor1A 15
#define motor1B 2
#define motor2A 4
#define motor2B 16

// GYROSCOPE VARIABLES
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

int readChannel(int channel_input){
  int value = pulseIn(channel_input, HIGH, 30000);
  if (value == 0){
    return 50;
  }
  else{
    return map(value, 1000, 1950, 0, 100);
  }
  
}

void initSensors()
{
  if(!accel.begin())
  {
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void setup(void)
{
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);

  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); 
  Serial.println("");
  
  initSensors();  // iniciar sensor
}

void gyroFunction(){
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  // leer el estado de los sensores
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  // usar el algoritmo de fusion de la libreria para combinar las mediciones
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));
  }
}

void loop(void)
{
  valueCH1 = readChannel(CH1);
  valueCH2 = readChannel(CH2);
  valueCH3 = readChannel(CH3);
  valueCH4 = readChannel(CH4);
  valueCH5 = readChannel(CH5);
  valueCH6 = readChannel(CH6);
  Serial.print("CH1 ");
  Serial.print(valueCH1);
  Serial.print(" - CH2 ");
  Serial.print(valueCH2);
  Serial.print(" - CH3 ");
  Serial.print(valueCH3);
  Serial.print(" - CH4 ");
  Serial.print(valueCH4);
  Serial.print(" - CH5 ");
  Serial.print(valueCH5);
  Serial.print(" - CH6 ");
  Serial.println(valueCH6);

  if (valueCH2 > 50){
      if (valueCH3 > 70){
      Serial.println("FORWARD");
      digitalWrite(motor1A, 0);
      digitalWrite(motor1B, 255);
      digitalWrite(motor2A, 0);
      digitalWrite(motor2B, 255);
      }
      else if (valueCH3 < 30){
      Serial.println("BACKWARD");
      digitalWrite(motor1A, 255);
      digitalWrite(motor1B, 0);
      digitalWrite(motor2A, 255);
      digitalWrite(motor2B, 0);
      }
      else if (valueCH6 > 70){
      Serial.println("RIGHT");
      digitalWrite(motor1A, 0);
      digitalWrite(motor1B, 255);
      digitalWrite(motor2A, 255);
      digitalWrite(motor2B, 0);
      }
      else if (valueCH6 < 30){
      Serial.println("LEFT");
      digitalWrite(motor1A, 255);
      digitalWrite(motor1B, 0);
      digitalWrite(motor2A, 0);
      digitalWrite(motor2B, 255);
      }

      else if (valueCH3 < 70 && valueCH3 > 30 && valueCH6 < 70 && valueCH6>30){
        Serial.println("STOP");
        digitalWrite(motor1A, 0);
        digitalWrite(motor1B, 0);
        digitalWrite(motor2A, 0);
        digitalWrite(motor2B, 0);
      }
  }
  
}

void motorController(){
  //FORWARD
  digitalWrite(motor1A, 0);
  digitalWrite(motor1B, 255);
  digitalWrite(motor2A, 0);
  digitalWrite(motor2B, 255);

  //RIGHT
  digitalWrite(motor1A, 0);
  digitalWrite(motor1B, 255);
  digitalWrite(motor2A, 255);
  digitalWrite(motor2B, 0);
}